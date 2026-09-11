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
#include "control_signal_factory.h"

#include <rv2_interfaces/control_signal_info.h>
#include <rv2_interfaces/service.h>
#include <rv2_interfaces/srv/control_signal_reg.hpp>
#include <rv2_interfaces/srv/control_signal_info_req.hpp>

#include <typeindex>
#include <typeinfo>

namespace rv2_interfaces
{

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
    ControlSignalManager(rclcpp::Node* node, const std::string& name, int64_t statusTimerIntervalMs = 1000) :
        node_(node),
        name_(name)
    {
        using namespace std::placeholders;

        regSrv_ = node_->create_service<srv::ControlSignalReg>(name_ + "/control_signal_reg",
                                                               std::bind(&ControlSignalManager::_onReg, this, _1, _2));

        infoReqSrv_ = node_->create_service<srv::ControlSignalInfoReq>(
            name_ + "/control_signal_info_req", std::bind(&ControlSignalManager::_onInfoReq, this, _1, _2));

        // Low-frequency status timer: checks Source/Sink states and removes entries
        // that have been continuously in TIMEOUT for longer than their disconnect_timeout_ns.
        statusTimer_ = node_->create_wall_timer(std::chrono::milliseconds(statusTimerIntervalMs),
                                                [this]()
                                                {
                                                    _statusTimerCb();
                                                });

        RCLCPP_INFO(node_->get_logger(),
                    "[CSM:%s] Started. Services: '%s/control_signal_reg', '%s/control_signal_info_req'",
                    name_.c_str(),
                    name_.c_str(),
                    name_.c_str());
    }

    // Non-copyable, non-movable (owns ROS 2 service handles).
    ControlSignalManager(const ControlSignalManager&) = delete;
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
                             name_.c_str(),
                             info.channel_name.c_str(),
                             v.error.c_str());
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
                            name_.c_str(),
                            info.channel_name.c_str());
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
                         name_.c_str(),
                         svcName.c_str(),
                         static_cast<long>(timeoutMs));
            return false;
        }

        // Send registration request.
        auto req = std::make_shared<srv::ControlSignalReg::Request>();
        req->source_csm_name = name_;
        req->control_signal_source_info = info;

        auto future = client->async_send_request(req);
        if (future.wait_for(std::chrono::milliseconds(timeoutMs)) != std::future_status::ready)
        {
            RCLCPP_ERROR(node_->get_logger(),
                         "[CSM:%s] registerSource: no response from '%s' within %ldms",
                         name_.c_str(),
                         svcName.c_str(),
                         static_cast<long>(timeoutMs));
            return false;
        }

        auto res = future.get();
        if (res->response != SRV_RES_SUCCESS)
        {
            RCLCPP_ERROR(node_->get_logger(),
                         "[CSM:%s] registerSource: rejected by '%s': %s",
                         name_.c_str(),
                         info.target_csm_name.c_str(),
                         res->reason.c_str());
            return false;
        }

        // Target CSM accepted — create and store the local Source.
        auto source = _makeSource(info);
        if (!source)
        {
            RCLCPP_ERROR(node_->get_logger(),
                         "[CSM:%s] registerSource: unsupported type='%s' or mode='%s'",
                         name_.c_str(),
                         info.control_signal_type.c_str(),
                         info.control_signal_mode.c_str());
            return false;
        }

        {
            std::lock_guard<std::mutex> lk(sourceMtx_);
            sources_[info.channel_name] = source;
        }

        RCLCPP_INFO(node_->get_logger(),
                    "[CSM:%s] Source registered: ch='%s' mode='%s' type='%s' target='%s'",
                    name_.c_str(),
                    info.channel_name.c_str(),
                    info.control_signal_mode.c_str(),
                    info.control_signal_type.c_str(),
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
        if (it == sources_.end())
            return ControlSignalState::UNKNOWN;
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
        if (it == sinks_.end())
            return ControlSignalState::UNKNOWN;
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
     *
     * Resolves the type key via ControlSignalFactory; returns
     * CONTROL_SIGNAL_TYPE_UNKNOWN if msgT has not been registered.
     */
    template <typename msgT> static std::string typeKeyFor()
    {
        const std::string key = ControlSignalFactory::Instance().typeKey(std::type_index(typeid(msgT)));
        return key.empty() ? msg::ControlSignalConst::CONTROL_SIGNAL_TYPE_UNKNOWN : key;
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
    template <typename msgT> void setSinkMsgCallback(std::function<void(const msgT&, const msg::ControlSignalInfo&)> cb)
    {
        const std::type_index tid(typeid(msgT));

        // Store the erased applicator. It installs a type-erased callback on the
        // Sink (via BaseControlSignalSink::setErasedMsgCallback) that restores
        // the concrete msgT before invoking the user callback — no knowledge of
        // the concrete Sink specialisation is required.
        {
            std::lock_guard<std::mutex> lk(cbMtx_);
            if (cb)
            {
                typedCbs_[tid] =
                    [cb](std::shared_ptr<BaseControlSignalSink> base, const msg::ControlSignalInfo& /*info*/)
                {
                    base->setErasedMsgCallback(
                        [cb](const void* m, const msg::ControlSignalInfo& i)
                        {
                            cb(*static_cast<const msgT*>(m), i);
                        });
                };
            }
            else
            {
                typedCbs_.erase(tid);
            }
        }

        // Apply to already-created Sinks of this type.
        std::lock_guard<std::mutex> lk(sinkMtx_);
        for (auto& [ch, snk] : sinks_)
        {
            if (snk->msgType() == tid)
                _applyCbToSink(snk, tid);
        }
    }

private:
    rclcpp::Node* node_;
    std::string name_;

    mutable std::mutex sourceMtx_;
    std::map<std::string, std::shared_ptr<BaseControlSignalSource>> sources_;  // key: channel_name
    std::map<std::string, int64_t>
        sourceTimeoutSinceNs_;  // key: channel_name, value: steadyNs() when TIMEOUT first observed

    mutable std::mutex sinkMtx_;
    std::map<std::string, std::shared_ptr<BaseControlSignalSink>> sinks_;  // key: channel_name
    std::map<std::string, int64_t>
        sinkTimeoutSinceNs_;  // key: channel_name, value: steadyNs() when TIMEOUT first observed

    // Per-type sink callbacks: key = std::type_index of the concrete msgT.
    // Value is a type-erased applicator that installs an erased callback on the
    // Sink (see setSinkMsgCallback / BaseControlSignalSink::setErasedMsgCallback).
    using CbApplicator = std::function<void(std::shared_ptr<BaseControlSignalSink>, const msg::ControlSignalInfo&)>;
    mutable std::mutex cbMtx_;
    std::map<std::type_index, CbApplicator> typedCbs_;

    // Apply the registered callback (if any) for the sink's message type.
    // Must be called with sinkMtx_ held (cbMtx_ acquired internally).
    void _applyCbToSink(std::shared_ptr<BaseControlSignalSink>& snk, std::type_index tid)
    {
        CbApplicator applicator;
        {
            std::lock_guard<std::mutex> lk(cbMtx_);
            auto it = typedCbs_.find(tid);
            if (it == typedCbs_.end())
                return;
            applicator = it->second;
        }
        if (applicator)
            applicator(snk, snk->getInfo());
    }

    rclcpp::Service<srv::ControlSignalReg>::SharedPtr regSrv_;
    rclcpp::Service<srv::ControlSignalInfoReq>::SharedPtr infoReqSrv_;
    rclcpp::TimerBase::SharedPtr statusTimer_;

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
                const ControlSignalState st = src->getState();
                const int64_t discTo = src->getInfo().disconnect_timeout_ns;

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
                                    name_.c_str(),
                                    ch.c_str(),
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
                const ControlSignalState st = snk->getState();
                const int64_t discTo = snk->getInfo().disconnect_timeout_ns;

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
                                    name_.c_str(),
                                    ch.c_str(),
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
                     name_.c_str(),
                     sources_.size(),
                     sinks_.size());
    }

    // ── ControlSignalReg service callback ──────────────────────────────────────
    /**
     * Receives a source registration from a remote CSM.
     * Creates a matching Sink and stores it. Rejects duplicates.
     */
    void _onReg(const std::shared_ptr<srv::ControlSignalReg::Request> req,
                std::shared_ptr<srv::ControlSignalReg::Response> res)
    {
        const auto& info = req->control_signal_source_info;

        // Validate configuration fields.
        {
            const auto v = validateControlSignalInfo(info);
            if (!v.valid)
            {
                res->response = SRV_RES_ERROR;
                res->reason = "Invalid ControlSignalInfo: " + v.error;
                RCLCPP_ERROR(node_->get_logger(),
                             "[CSM:%s] _onReg from '%s': %s",
                             name_.c_str(),
                             req->source_csm_name.c_str(),
                             res->reason.c_str());
                return;
            }
        }

        {
            std::lock_guard<std::mutex> lk(sinkMtx_);
            if (sinks_.count(info.channel_name))
            {
                res->response = SRV_RES_IGNORED;
                res->reason = "Sink already exists for channel: " + info.channel_name;
                RCLCPP_WARN(node_->get_logger(),
                            "[CSM:%s] _onReg from '%s': %s",
                            name_.c_str(),
                            req->source_csm_name.c_str(),
                            res->reason.c_str());
                return;
            }
        }

        auto sink = _makeSink(info);
        if (!sink)
        {
            res->response = SRV_RES_ERROR;
            res->reason = "Unsupported type='" + info.control_signal_type + "' mode='" + info.control_signal_mode + "'";
            RCLCPP_ERROR(node_->get_logger(),
                         "[CSM:%s] _onReg from '%s': %s",
                         name_.c_str(),
                         req->source_csm_name.c_str(),
                         res->reason.c_str());
            return;
        }

        {
            std::lock_guard<std::mutex> lk(sinkMtx_);
            sinks_[info.channel_name] = sink;
            _applyCbToSink(sinks_[info.channel_name], sinks_[info.channel_name]->msgType());
        }

        res->response = SRV_RES_SUCCESS;
        if (info.use_keep_alive && info.keep_alive_interval_ns > 0)
            res->keep_alive_topic_name = info.channel_name + "_keep_alive";

        RCLCPP_INFO(node_->get_logger(),
                    "[CSM:%s] Sink created from source CSM '%s': ch='%s' mode='%s' type='%s'",
                    name_.c_str(),
                    req->source_csm_name.c_str(),
                    info.channel_name.c_str(),
                    info.control_signal_mode.c_str(),
                    info.control_signal_type.c_str());
    }

    // ── ControlSignalInfoReq service callback ──────────────────────────────────
    /**
     * Returns the ControlSignalInfo for all managed Sources and Sinks.
     */
    void _onInfoReq(const std::shared_ptr<srv::ControlSignalInfoReq::Request> /*req*/,
                    std::shared_ptr<srv::ControlSignalInfoReq::Response> res)
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

    // ── Dynamic factories (delegated to ControlSignalFactory) ────────────────

    /**
     * Creates a ControlSignalSource for @p info via ControlSignalFactory.
     * Returns nullptr if the type is not registered.
     */
    std::shared_ptr<BaseControlSignalSource> _makeSource(const msg::ControlSignalInfo& info)
    {
        try
        {
            return ControlSignalFactory::Instance().CreateSource(info.control_signal_type, node_, info);
        }
        catch (const std::runtime_error&)
        {
            return nullptr;
        }
    }

    /**
     * Creates a ControlSignalSink for @p info via ControlSignalFactory.
     * Returns nullptr if the type is not registered.
     */
    std::shared_ptr<BaseControlSignalSink> _makeSink(const msg::ControlSignalInfo& info)
    {
        try
        {
            return ControlSignalFactory::Instance().CreateSink(info.control_signal_type, node_, info);
        }
        catch (const std::runtime_error&)
        {
            return nullptr;
        }
    }
};

}  // namespace rv2_interfaces
