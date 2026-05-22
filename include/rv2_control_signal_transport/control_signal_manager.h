/**
 * ControlSignalManager (CSM) — manages ControlSignalSources and ControlSignalSinks
 * for a ROS 2 node, and provides the inter-CSM registration protocol.
 *
 * Hosted ROS 2 services (both on "<name>/<service_name>"):
 *  - <name>/control_signal_reg      (rv2_interfaces::srv::ControlSignalReg)
 *      Receives source registrations from other CSMs and creates matching Sinks.
 *  - <name>/control_signal_info_req (rv2_interfaces::srv::ControlSignalInfoReq)
 *      Returns the ControlSignalInfo list for all managed Sources and Sinks.
 *
 * API:
 *  - registerSource(info): creates a local Source, then calls the target CSM's
 *    control_signal_reg service so the remote side creates the matching Sink.
 *
 * Threading requirements:
 *  The parent node must be spinning (e.g. via rclcpp::spin or a MultiThreadedExecutor)
 *  in a separate thread before registerSource() is called, because registerSource()
 *  blocks waiting for the target CSM's service response.
 */

#pragma once

#include "control_signal_transport.h"

#include <rv2_interfaces/control_signal_info.h>
#include <rv2_interfaces/service.h>
#include <rv2_interfaces/srv/control_signal_reg.hpp>
#include <rv2_interfaces/srv/control_signal_info_req.hpp>
#include <rv2_interfaces/srv/control_signal_joy.hpp>
#include <rv2_interfaces/srv/control_signal_twist.hpp>


namespace rv2_interfaces
{

// ── Trait: maps msgT → matching ROS 2 service type (or void) ─────────────────
namespace detail
{
template<typename T> struct ServiceTypeOf          { using type = void; };
template<> struct ServiceTypeOf<sensor_msgs::msg::Joy>     { using type = srv::ControlSignalJoy; };
template<> struct ServiceTypeOf<geometry_msgs::msg::Twist> { using type = srv::ControlSignalTwist; };
// std_msgs::msg::String → void (no matching srv type)
} // namespace detail


/**
 * @brief ControlSignalManager manages a set of Sources and Sinks for a node.
 *
 * Sources are registered via registerSource(); the manager calls the target
 * CSM's ControlSignalReg service to create the remote Sink, then stores the
 * local Source on success.
 *
 * Sinks are created automatically when this CSM's ControlSignalReg service
 * server receives a registration request from a remote Source CSM.
 *
 * Both Sources and Sinks are keyed by channel_name; duplicate registrations
 * for the same channel are rejected.
 */
class ControlSignalManager
{
public:
    /**
     * @brief Construct a ControlSignalManager and advertise its two service servers.
     *
     * @param node                    Parent ROS 2 node (must outlive this object).
     * @param name                    Unique manager name; used as prefix for hosted service names.
     * @param statusTimerIntervalMs   Period of the low-frequency status/disconnect timer in ms (default 1000).
     */
    ControlSignalManager(rclcpp::Node* node, const std::string& name,
                         int64_t statusTimerIntervalMs = 1000)
        : node_(node)
        , name_(name)
    {
        using namespace std::placeholders;

        regSrv_ = node_->create_service<srv::ControlSignalReg>(
            name_ + "/control_signal_reg",
            std::bind(&ControlSignalManager::_onReg, this, _1, _2));

        infoReqSrv_ = node_->create_service<srv::ControlSignalInfoReq>(
            name_ + "/control_signal_info_req",
            std::bind(&ControlSignalManager::_onInfoReq, this, _1, _2));

        // Low-frequency status timer: checks Source/Sink states and removes entries
        // that have been continuously in TIMEOUT for longer than their disconnect_timeout_ns.
        statusTimer_ = node_->create_wall_timer(
            std::chrono::milliseconds(statusTimerIntervalMs),
            [this]() { _statusTimerCb(); });

        RCLCPP_INFO(node_->get_logger(),
            "[CSM:%s] Started. Services: '%s/control_signal_reg', '%s/control_signal_info_req'",
            name_.c_str(), name_.c_str(), name_.c_str());
    }

    // Non-copyable, non-movable (owns ROS 2 service handles).
    ControlSignalManager(const ControlSignalManager&)            = delete;
    ControlSignalManager& operator=(const ControlSignalManager&) = delete;

    // ── Public API ─────────────────────────────────────────────────────────────

    /**
     * @brief Register a control signal source.
     *
     * Steps:
     *  1. Calls info.target_csm_name/control_signal_reg on the target CSM.
     *  2. On acceptance, creates a local ControlSignalSource from info.
     *  3. Stores the Source; returns true.
     *
     * If the target CSM is unreachable, times out, or rejects the registration,
     * no local Source is created and false is returned.
     *
     * @note The parent node must already be spinning in another thread.
     *       Calling registerSource() from within a ROS 2 callback on a
     *       single-threaded executor will deadlock.
     *
     * @param info       Source configuration. info.target_csm_name must name the
     *                   remote CSM that will host the matching Sink.
     * @param timeoutMs  Maximum wall time (ms) to wait for the target CSM's response.
     * @return true on successful registration; false otherwise.
     */
    bool registerSource(const msg::ControlSignalInfo& info, int64_t timeoutMs = 5000)
    {
        // Validate configuration fields before doing anything else.
        {
            const auto v = validateControlSignalInfo(info);
            if (!v.valid)
            {
                RCLCPP_ERROR(node_->get_logger(),
                    "[CSM:%s] registerSource: invalid ControlSignalInfo for channel '%s': %s",
                    name_.c_str(), info.channel_name.c_str(), v.error.c_str());
                return false;
            }
        }

        // Reject duplicate channel registration.
        {
            std::lock_guard<std::mutex> lk(sourceMtx_);
            if (sources_.count(info.channel_name))
            {
                RCLCPP_WARN(node_->get_logger(),
                    "[CSM:%s] registerSource: source already registered for channel '%s'",
                    name_.c_str(), info.channel_name.c_str());
                return false;
            }
        }

        // Create a service client for the target CSM's registration endpoint.
        const std::string svcName = info.target_csm_name + "/control_signal_reg";
        auto client = node_->create_client<srv::ControlSignalReg>(svcName);

        if (!client->wait_for_service(std::chrono::milliseconds(timeoutMs)))
        {
            RCLCPP_ERROR(node_->get_logger(),
                "[CSM:%s] registerSource: service '%s' not available within %ldms",
                name_.c_str(), svcName.c_str(), static_cast<long>(timeoutMs));
            return false;
        }

        // Send registration request.
        auto req = std::make_shared<srv::ControlSignalReg::Request>();
        req->source_csm_name           = name_;
        req->control_signal_source_info = info;

        auto future = client->async_send_request(req);
        if (future.wait_for(std::chrono::milliseconds(timeoutMs)) != std::future_status::ready)
        {
            RCLCPP_ERROR(node_->get_logger(),
                "[CSM:%s] registerSource: no response from '%s' within %ldms",
                name_.c_str(), svcName.c_str(), static_cast<long>(timeoutMs));
            return false;
        }

        auto res = future.get();
        if (res->response != SRV_RES_SUCCESS)
        {
            RCLCPP_ERROR(node_->get_logger(),
                "[CSM:%s] registerSource: rejected by '%s': %s",
                name_.c_str(), info.target_csm_name.c_str(), res->reason.c_str());
            return false;
        }

        // Target CSM accepted — create and store the local Source.
        auto source = _makeSource(info);
        if (!source)
        {
            RCLCPP_ERROR(node_->get_logger(),
                "[CSM:%s] registerSource: unsupported type='%s' or mode='%s'",
                name_.c_str(), info.control_signal_type.c_str(),
                info.control_signal_mode.c_str());
            return false;
        }

        {
            std::lock_guard<std::mutex> lk(sourceMtx_);
            sources_[info.channel_name] = source;
        }

        RCLCPP_INFO(node_->get_logger(),
            "[CSM:%s] Source registered: ch='%s' mode='%s' type='%s' target='%s'",
            name_.c_str(), info.channel_name.c_str(),
            info.control_signal_mode.c_str(), info.control_signal_type.c_str(),
            info.target_csm_name.c_str());
        return true;
    }

    // ── State queries ──────────────────────────────────────────────────────────

    /**
     * @brief Returns the state of a managed Source by channel_name.
     * Returns UNKNOWN if no source with that channel_name is managed.
     */
    ControlSignalState getSourceState(const std::string& channelName) const
    {
        std::lock_guard<std::mutex> lk(sourceMtx_);
        auto it = sources_.find(channelName);
        if (it == sources_.end()) return ControlSignalState::UNKNOWN;
        return it->second->getState();
    }

    /**
     * @brief Returns the state of a managed Sink by channel_name.
     * Returns UNKNOWN if no sink with that channel_name is managed.
     */
    ControlSignalState getSinkState(const std::string& channelName) const
    {
        std::lock_guard<std::mutex> lk(sinkMtx_);
        auto it = sinks_.find(channelName);
        if (it == sinks_.end()) return ControlSignalState::UNKNOWN;
        return it->second->getState();
    }

    // ── Info accessors ─────────────────────────────────────────────────────────

    /** @brief Returns the ControlSignalInfo for all managed Sources. */
    std::vector<msg::ControlSignalInfo> getSourceInfoList() const
    {
        std::lock_guard<std::mutex> lk(sourceMtx_);
        std::vector<msg::ControlSignalInfo> list;
        list.reserve(sources_.size());
        for (const auto& [ch, src] : sources_)
            list.push_back(src->getInfo());
        return list;
    }

    /** @brief Returns the ControlSignalInfo for all managed Sinks. */
    std::vector<msg::ControlSignalInfo> getSinkInfoList() const
    {
        std::lock_guard<std::mutex> lk(sinkMtx_);
        std::vector<msg::ControlSignalInfo> list;
        list.reserve(sinks_.size());
        for (const auto& [ch, snk] : sinks_)
            list.push_back(snk->getInfo());
        return list;
    }

    /**
     * @brief Returns a shared pointer to a managed Source by channel_name.
     * Returns nullptr if not found.
     */
    std::shared_ptr<BaseControlSignalSource> getSource(const std::string& channelName) const
    {
        std::lock_guard<std::mutex> lk(sourceMtx_);
        auto it = sources_.find(channelName);
        return (it != sources_.end()) ? it->second : nullptr;
    }

    /**
     * @brief Returns a shared pointer to a managed Sink by channel_name.
     * Returns nullptr if not found.
     */
    std::shared_ptr<BaseControlSignalSink> getSink(const std::string& channelName) const
    {
        std::lock_guard<std::mutex> lk(sinkMtx_);
        auto it = sinks_.find(channelName);
        return (it != sinks_.end()) ? it->second : nullptr;
    }

    /** @brief Returns this manager's name. */
    const std::string& getName() const { return name_; }

    /**
     * @brief Returns the ControlSignalConst type-string for a message type msgT.
     * Public static utility used by ControlServer and other consumers.
     */
    template<typename msgT>
    static std::string typeKeyFor()
    {
        if constexpr (std::is_same_v<msgT, sensor_msgs::msg::Joy>)
            return msg::ControlSignalConst::CONTROL_SIGNAL_TYPE_JOY;
        else if constexpr (std::is_same_v<msgT, geometry_msgs::msg::Twist>)
            return msg::ControlSignalConst::CONTROL_SIGNAL_TYPE_TWIST;
        else if constexpr (std::is_same_v<msgT, std_msgs::msg::String>)
            return msg::ControlSignalConst::CONTROL_SIGNAL_TYPE_STRING;
        else
            return msg::ControlSignalConst::CONTROL_SIGNAL_TYPE_UNKNOWN;
    }

    // ── Per-type sink message callback registration ────────────────────────────

    /**
     * @brief Register a callback invoked each time a Sink of type msgT stores a message.
     *
     * If a Sink for the given type already exists (past registrations), the callback
     * is applied immediately to those Sinks as well.
     * Replaces any previously registered callback for this type.
     *
     * The callback signature must be:
     *   void(const msgT&, const rv2_interfaces::msg::ControlSignalInfo&)
     *
     * Thread-safe; may be called before or after Sinks are created.
     *
     * @tparam msgT  ROS 2 message type (sensor_msgs::msg::Joy, geometry_msgs::msg::Twist, etc.)
     * @param  cb    Callback to invoke on each received message (nullptr clears the callback).
     */
    template<typename msgT>
    void setSinkMsgCallback(
        std::function<void(const msgT&, const msg::ControlSignalInfo&)> cb)
    {
        const std::string typeKey = _typeKey<msgT>();

        // Store the erased callback.
        {
            std::lock_guard<std::mutex> lk(cbMtx_);
            if (cb)
                typedCbs_[typeKey] = [cb](std::shared_ptr<BaseControlSignalSink> base,
                                          const msg::ControlSignalInfo& info)
                {
                    // Down-cast to the two concrete Sink types that match msgT.
                    if (auto* s = dynamic_cast<ControlSignalSink<msgT, void>*>(base.get()))
                        s->setMsgCallback([cb, info](const msgT& m, const msg::ControlSignalInfo& i) { cb(m, i); });
                    else if (auto* s = dynamic_cast<ControlSignalSink<msgT, srv::ControlSignalJoy>*>(base.get()))
                        s->setMsgCallback([cb, info](const msgT& m, const msg::ControlSignalInfo& i) { cb(m, i); });
                    else if (auto* s = dynamic_cast<ControlSignalSink<msgT, srv::ControlSignalTwist>*>(base.get()))
                        s->setMsgCallback([cb, info](const msgT& m, const msg::ControlSignalInfo& i) { cb(m, i); });
                    (void)info;
                };
            else
                typedCbs_.erase(typeKey);
        }

        // Apply to already-created Sinks of this type.
        std::lock_guard<std::mutex> lk(sinkMtx_);
        for (auto& [ch, snk] : sinks_)
        {
            if (snk->getInfo().control_signal_type == typeKey)
                _applyCbToSink(snk, typeKey);
        }
    }
private:
    rclcpp::Node* node_;
    std::string   name_;

    mutable std::mutex sourceMtx_;
    std::map<std::string, std::shared_ptr<BaseControlSignalSource>> sources_; // key: channel_name
    std::map<std::string, int64_t> sourceTimeoutSinceNs_; // key: channel_name, value: steadyNs() when TIMEOUT first observed

    mutable std::mutex sinkMtx_;
    std::map<std::string, std::shared_ptr<BaseControlSignalSink>> sinks_;     // key: channel_name
    std::map<std::string, int64_t> sinkTimeoutSinceNs_;   // key: channel_name, value: steadyNs() when TIMEOUT first observed

    // Per-type sink callbacks: key = control_signal_type string.
    // Value is a type-erased applicator that down-casts and calls setMsgCallback().
    using CbApplicator = std::function<void(std::shared_ptr<BaseControlSignalSink>,
                                            const msg::ControlSignalInfo&)>;
    mutable std::mutex                    cbMtx_;
    std::map<std::string, CbApplicator>   typedCbs_;

    // Returns the ControlSignalConst type string for a given msgT.
    template<typename msgT>
    static std::string _typeKey() { return typeKeyFor<msgT>(); }

    // Apply the registered callback (if any) for the sink's type.
    // Must be called with sinkMtx_ held (cbMtx_ acquired internally).
    void _applyCbToSink(std::shared_ptr<BaseControlSignalSink>& snk,
                        const std::string& typeKey)
    {
        CbApplicator applicator;
        {
            std::lock_guard<std::mutex> lk(cbMtx_);
            auto it = typedCbs_.find(typeKey);
            if (it == typedCbs_.end()) return;
            applicator = it->second;
        }
        if (applicator) applicator(snk, snk->getInfo());
    }

    rclcpp::Service<srv::ControlSignalReg>::SharedPtr     regSrv_;
    rclcpp::Service<srv::ControlSignalInfoReq>::SharedPtr infoReqSrv_;
    rclcpp::TimerBase::SharedPtr                          statusTimer_;

    // ── 1 Hz status timer callback ────────────────────────────────────────────
    /**
     * Runs every second. For each Source and Sink:
     *   - If state is TIMEOUT and disconnect_timeout_ns > 0:
     *       - Records the first time TIMEOUT was observed for that channel.
     *       - Removes the entry once it has been continuously in TIMEOUT
     *         for longer than disconnect_timeout_ns.
     *   - If state recovers (ACTIVE/UNKNOWN), resets the TIMEOUT tracking.
     */
    void _statusTimerCb()
    {
        const int64_t now = detail::steadyNs();

        // ── Sources ───────────────────────────────────────────────────────────
        {
            std::lock_guard<std::mutex> lk(sourceMtx_);
            std::vector<std::string> toErase;

            for (auto& [ch, src] : sources_)
            {
                const ControlSignalState st    = src->getState();
                const int64_t            discTo = src->getInfo().disconnect_timeout_ns;

                if (st == ControlSignalState::TIMEOUT && discTo > 0)
                {
                    auto it = sourceTimeoutSinceNs_.find(ch);
                    if (it == sourceTimeoutSinceNs_.end())
                    {
                        sourceTimeoutSinceNs_[ch] = now;  // first observation
                    }
                    else if (now - it->second > discTo)
                    {
                        RCLCPP_WARN(node_->get_logger(),
                            "[CSM:%s] Source '%s' has been in TIMEOUT for >%ld ms. Removing.",
                            name_.c_str(), ch.c_str(),
                            static_cast<long>(discTo / 1'000'000));
                        src->markDisconnected();
                        toErase.push_back(ch);
                    }
                }
                else
                {
                    sourceTimeoutSinceNs_.erase(ch);  // recovered or still UNKNOWN
                }
            }

            for (const auto& ch : toErase)
            {
                sources_.erase(ch);
                sourceTimeoutSinceNs_.erase(ch);
            }
        }

        // ── Sinks ─────────────────────────────────────────────────────────────
        {
            std::lock_guard<std::mutex> lk(sinkMtx_);
            std::vector<std::string> toErase;

            for (auto& [ch, snk] : sinks_)
            {
                const ControlSignalState st    = snk->getState();
                const int64_t            discTo = snk->getInfo().disconnect_timeout_ns;

                if (st == ControlSignalState::TIMEOUT && discTo > 0)
                {
                    auto it = sinkTimeoutSinceNs_.find(ch);
                    if (it == sinkTimeoutSinceNs_.end())
                    {
                        sinkTimeoutSinceNs_[ch] = now;  // first observation
                    }
                    else if (now - it->second > discTo)
                    {
                        RCLCPP_WARN(node_->get_logger(),
                            "[CSM:%s] Sink '%s' has been in TIMEOUT for >%ld ms. Removing.",
                            name_.c_str(), ch.c_str(),
                            static_cast<long>(discTo / 1'000'000));
                        snk->markDisconnected();
                        toErase.push_back(ch);
                    }
                }
                else
                {
                    sinkTimeoutSinceNs_.erase(ch);  // recovered or still UNKNOWN
                }
            }

            for (const auto& ch : toErase)
            {
                sinks_.erase(ch);
                sinkTimeoutSinceNs_.erase(ch);
            }
        }

        RCLCPP_DEBUG(node_->get_logger(),
            "[CSM:%s] Status: %zu source(s), %zu sink(s)",
            name_.c_str(), sources_.size(), sinks_.size());
    }

    // ── ControlSignalReg service callback ──────────────────────────────────────
    /**
     * Receives a source registration from a remote CSM.
     * Creates a matching Sink and stores it. Rejects duplicates.
     */
    void _onReg(
        const std::shared_ptr<srv::ControlSignalReg::Request>  req,
        std::shared_ptr<srv::ControlSignalReg::Response>       res)
    {
        const auto& info = req->control_signal_source_info;

        // Validate configuration fields.
        {
            const auto v = validateControlSignalInfo(info);
            if (!v.valid)
            {
                res->response = SRV_RES_ERROR;
                res->reason   = "Invalid ControlSignalInfo: " + v.error;
                RCLCPP_ERROR(node_->get_logger(),
                    "[CSM:%s] _onReg from '%s': %s",
                    name_.c_str(), req->source_csm_name.c_str(), res->reason.c_str());
                return;
            }
        }

        {
            std::lock_guard<std::mutex> lk(sinkMtx_);
            if (sinks_.count(info.channel_name))
            {
                res->response = SRV_RES_IGNORED;
                res->reason   = "Sink already exists for channel: " + info.channel_name;
                RCLCPP_WARN(node_->get_logger(),
                    "[CSM:%s] _onReg from '%s': %s",
                    name_.c_str(), req->source_csm_name.c_str(), res->reason.c_str());
                return;
            }
        }

        auto sink = _makeSink(info);
        if (!sink)
        {
            res->response = SRV_RES_ERROR;
            res->reason   = "Unsupported type='" + info.control_signal_type +
                            "' mode='" + info.control_signal_mode + "'";
            RCLCPP_ERROR(node_->get_logger(),
                "[CSM:%s] _onReg from '%s': %s",
                name_.c_str(), req->source_csm_name.c_str(), res->reason.c_str());
            return;
        }

        {
            std::lock_guard<std::mutex> lk(sinkMtx_);
            sinks_[info.channel_name] = sink;
            _applyCbToSink(sinks_[info.channel_name], info.control_signal_type);
        }

        res->response = SRV_RES_SUCCESS;
        if (info.use_keep_alive && info.keep_alive_interval_ns > 0)
            res->keep_alive_topic_name = info.channel_name + "_keep_alive";

        RCLCPP_INFO(node_->get_logger(),
            "[CSM:%s] Sink created from source CSM '%s': ch='%s' mode='%s' type='%s'",
            name_.c_str(), req->source_csm_name.c_str(),
            info.channel_name.c_str(),
            info.control_signal_mode.c_str(), info.control_signal_type.c_str());
    }

    // ── ControlSignalInfoReq service callback ──────────────────────────────────
    /**
     * Returns the ControlSignalInfo for all managed Sources and Sinks.
     */
    void _onInfoReq(
        const std::shared_ptr<srv::ControlSignalInfoReq::Request>  /*req*/,
        std::shared_ptr<srv::ControlSignalInfoReq::Response>        res)
    {
        {
            std::lock_guard<std::mutex> lk(sourceMtx_);
            res->source_list.reserve(sources_.size());
            for (const auto& [ch, src] : sources_)
                res->source_list.push_back(src->getInfo());
        }
        {
            std::lock_guard<std::mutex> lk(sinkMtx_);
            res->sink_list.reserve(sinks_.size());
            for (const auto& [ch, snk] : sinks_)
                res->sink_list.push_back(snk->getInfo());
        }
        res->response = SRV_RES_SUCCESS;
    }

    // ── Dynamic factories (dispatch on mode + type) ────────────────────────────

    /**
     * Creates a ControlSignalSource dispatching on both control_signal_mode and
     * control_signal_type.
     *
     * For service mode, the Source is created directly to ensure the msgT/srvT pair
     * is always compatible (Joy↔ControlSignalJoy, Twist↔ControlSignalTwist).
     * "string" service mode is not defined in rv2_interfaces; falls back to topic.
     */
    std::shared_ptr<BaseControlSignalSource> _makeSource(const msg::ControlSignalInfo& info)
    {
        const auto& mode = info.control_signal_mode;
        const auto& type = info.control_signal_type;

        if (mode == msg::ControlSignalConst::CONTROL_SIGNAL_MODE_SERVICE)
        {
            if (type == msg::ControlSignalConst::CONTROL_SIGNAL_TYPE_JOY)
                return std::make_shared<ControlSignalSource<sensor_msgs::msg::Joy,
                                                            srv::ControlSignalJoy>>(node_, info);
            if (type == msg::ControlSignalConst::CONTROL_SIGNAL_TYPE_TWIST)
                return std::make_shared<ControlSignalSource<geometry_msgs::msg::Twist,
                                                            srv::ControlSignalTwist>>(node_, info);
            // "string" has no service type — fall through to topic mode.
        }
        // Topic mode (also the fallback for unsupported service types).
        return makeControlSignalSource<void>(node_, info);
    }

    /**
     * Creates a ControlSignalSink dispatching on both control_signal_mode and
     * control_signal_type.
     *
     * Same direct-creation strategy as _makeSource() for service mode.
     */
    std::shared_ptr<BaseControlSignalSink> _makeSink(const msg::ControlSignalInfo& info)
    {
        const auto& mode = info.control_signal_mode;
        const auto& type = info.control_signal_type;

        if (mode == msg::ControlSignalConst::CONTROL_SIGNAL_MODE_SERVICE)
        {
            if (type == msg::ControlSignalConst::CONTROL_SIGNAL_TYPE_JOY)
                return std::make_shared<ControlSignalSink<sensor_msgs::msg::Joy,
                                                          srv::ControlSignalJoy>>(node_, info);
            if (type == msg::ControlSignalConst::CONTROL_SIGNAL_TYPE_TWIST)
                return std::make_shared<ControlSignalSink<geometry_msgs::msg::Twist,
                                                          srv::ControlSignalTwist>>(node_, info);
        }
        return makeControlSignalSink<void>(node_, info);
    }
};


} // namespace rv2_interfaces
