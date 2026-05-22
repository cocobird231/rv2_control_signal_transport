/**
 * This document defines an architecture for controlling signal data flow, consisting of Sources, 
 * Sinks, and a ControlSignalManager.
 * Sources and Sinks can be initially configured to operate in either topic mode or service mode, 
 * and both support a timeout mechanism.
 * The ControlSignalManager is responsible for managing the registration information of Sources 
 * and Sinks, and generating corresponding Sinks and Sources accordingly. It supports multiple 
 * registrations of Sources and Sinks and maintains their states, such as timeout status.
 * 
 * - ControlSignalSource: 
 *      Can act as a publisher (topic mode) or a client (service mode).
 * - ControlSignalSink:
 *      Can act as a subscriber (topic mode) or a server (service mode).
 * - ControlSignalManager:
 *      - Accepts API requests to register Sources and sends the registration information to a target ControlSignalManager via service calls
 *      - The target ControlSignalManager generates the corresponding Sink based on the registration information
 *      - Maintains the states of both Sources and Sinks
 */

#pragma once
#include <string>
#include <variant>
#include <vector>
#include <map>

#include <memory>
#include <chrono>
#include <atomic>
#include <mutex>
#include <functional>
#include <optional>

#include <rclcpp/rclcpp.hpp>
#include <rv2_interfaces/msg/control_signal_const.hpp>
#include <rv2_interfaces/msg/control_signal_info.hpp>
#include <rv2_interfaces/msg/service_response_status_const.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/string.hpp>


namespace rv2_interfaces
{


/**
 * @brief Runtime state of a ControlSignalSource or ControlSignalSink.
 * UNKNOWN  : Initial state; no activity has been observed yet.
 * ACTIVE   : Transport is operating normally (signal received / response succeeded).
 * TIMEOUT  : Was ACTIVE but no signal observed within the configured timeout window.
 */
enum class ControlSignalState
{
    UNKNOWN,      ///< No message received yet (initial state).
    ACTIVE,       ///< Receiving messages within expected rate.
    LOW_FREQ,     ///< Receiving messages but below expected rate (usable, lower priority).
    TIMEOUT,      ///< Was ACTIVE/LOW_FREQ but elapsed since last message > timeout threshold.
    DISCONNECTED  ///< Terminal: removed by CSM after prolonged TIMEOUT.
};



/**
 * @brief BaseControlSignalSource is an abstract base class representing a source of control signals.
 *
 * It can operate in either topic mode (publisher) or service mode (service client).
 * Derived classes are responsible for constructing the appropriate ROS 2 transport.
 */
class BaseControlSignalSource
{
public:
    BaseControlSignalSource() = default;
    virtual ~BaseControlSignalSource() = default;

    /**
     * @brief Returns the current runtime state of this source.
     *
     * Performs a passive timeout check (no background thread):
     *  - No keep-alive + topic mode  : always UNKNOWN (no liveness feedback).
     *  - No keep-alive + service mode: state is owned by send() — ACTIVE on
     *    success response, TIMEOUT on request timeout. getState() returns as-is.
     *  - keep-alive enabled           : ACTIVE while a keep-alive was received
     *    within 2 × keep_alive_interval_ns; transitions to TIMEOUT otherwise.
     */
    virtual ControlSignalState getState() const = 0;

    /**
     * @brief Returns the ControlSignalInfo this source was constructed from.
     */
    virtual const msg::ControlSignalInfo& getInfo() const = 0;

    /**
     * @brief Transition this source to the terminal DISCONNECTED state.
     *
     * Called by the CSM when the source has been in TIMEOUT longer than
     * disconnect_timeout_ns. After this call, getState() returns DISCONNECTED
     * and the CSM removes the source from its registry.
     */
    virtual void markDisconnected() = 0;
};



/**
 * @brief BaseControlSignalSink is an abstract base class representing a sink of control signals.
 *
 * It can operate in either topic mode (subscriber) or service mode (service server).
 * Derived classes are responsible for constructing the appropriate ROS 2 transport.
 */
class BaseControlSignalSink
{
public:
    BaseControlSignalSink() = default;
    virtual ~BaseControlSignalSink() = default;

    /**
     * @brief Returns the current runtime state of this sink.
     *
     * Performs a passive timeout check (no background thread):
     *  - UNKNOWN  : no message has been received yet (initial state).
     *  - ACTIVE   : at least one message received; last within timeout_ns.
     *  - TIMEOUT  : was ACTIVE but timeout_ns elapsed since last message.
     *    (timeout_ns == 0 disables the timeout check entirely.)
     */
    virtual ControlSignalState getState() const = 0;

    /**
     * @brief Returns the ControlSignalInfo this sink was constructed from.
     */
    virtual const msg::ControlSignalInfo& getInfo() const = 0;

    /**
     * @brief Transition this sink to the terminal DISCONNECTED state.
     *
     * Called by the CSM when the sink has been in TIMEOUT longer than
     * disconnect_timeout_ns. After this call, getState() returns DISCONNECTED
     * and the CSM removes the sink from its registry.
     */
    virtual void markDisconnected() = 0;
};



// ============================================================
//  Internal helper
// ============================================================

namespace detail
{
/// Returns steady-clock time as nanoseconds since epoch (lock-free, no allocation).
inline int64_t steadyNs() noexcept
{
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
}

/// Lazy trait: maps srvT → rclcpp::Client<srvT>::SharedPtr, or std::monostate when srvT=void.
/// Avoids instantiating rclcpp::Client<void> (which is ill-formed).
template<typename T>
struct ClientPtrOf { using type = typename rclcpp::Client<T>::SharedPtr; };
template<>
struct ClientPtrOf<void> { using type = std::monostate; };

/// Lazy trait: maps srvT → rclcpp::Service<srvT>::SharedPtr, or std::monostate when srvT=void.
template<typename T>
struct ServicePtrOf { using type = typename rclcpp::Service<T>::SharedPtr; };
template<>
struct ServicePtrOf<void> { using type = std::monostate; };

} // namespace detail


// ============================================================
//  ControlSignalSource
// ============================================================

/**
 * @brief ControlSignalSource<msgT, srvT> — typed control-signal source.
 *
 * Transport is a std::variant over Publisher (topic) or Client (service),
 * selected once at construction from ControlSignalInfo.
 *
 * State and the last-activity timestamp are held in lock-free atomics;
 * no mutex is used for state management.
 * Timeout is checked passively on send() and getState() — no background thread.
 *
 * Liveness rules (evaluated in getState() via _checkTimeout()):
 *
 *   ┌──────────────────┬────────────────────────────────┬─────────────────────────────────────┐
 *   │                  │ use_keep_alive = false         │ use_keep_alive = true               │
 *   ├──────────────────┼────────────────────────────────┼─────────────────────────────────────┤
 *   │ topic mode       │ UNKNOWN (no feedback)          │ ACTIVE if keep-alive received within │
 *   │                  │                                │ 2 × keep_alive_interval_ns;          │
 *   │                  │                                │ TIMEOUT otherwise                   │
 *   ├──────────────────┼────────────────────────────────┼─────────────────────────────────────┤
 *   │ service mode     │ ACTIVE on success response;    │ ACTIVE if keep-alive received within │
 *   │                  │ TIMEOUT on request timeout     │ 2 × keep_alive_interval_ns;          │
 *   │                  │ (state set directly in send()) │ TIMEOUT otherwise                   │
 *   └──────────────────┴────────────────────────────────┴─────────────────────────────────────┘
 *
 * Keep-alive (Source side): subscribes to <channel_name>_keep_alive
 *   (std_msgs::msg::String). Any received message calls _markActivity().
 *   Works in both topic and service modes.
 *
 * @tparam msgT  ROS 2 message type (e.g. sensor_msgs::msg::Joy).
 * @tparam srvT  ROS 2 service type.  Request must have a `data` field of type
 *               msgT; Response must have a `success` bool field.
 *               Defaults to void (topic-only).
 */
template<typename msgT, typename srvT = void>
class ControlSignalSource : public BaseControlSignalSource
{
private:
    rclcpp::Node*          node_;
    msg::ControlSignalInfo info_;
    bool                   isTopicMode_;    // cached at construction

    using PubPtr = typename rclcpp::Publisher<msgT>::SharedPtr;
    using CliPtr = typename detail::ClientPtrOf<srvT>::type;
    std::variant<PubPtr, CliPtr> transport_;

    // Keep-alive subscriber (both modes when use_keep_alive == true).
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr keepAliveSub_;

    // isTopicMode_ declared before state_ so the initializer list can use it safely.
    mutable std::atomic<ControlSignalState> state_;
    mutable std::atomic<int64_t>            lastActivityNs_;

    // Passive timeout — modifies mutable atomics; safe to call from const context.
    void _checkTimeout() const
    {
        // No keep-alive + topic mode: no liveness feedback possible → always UNKNOWN.
        // No keep-alive + service mode: state is driven entirely by send() results
        // (ACTIVE on success response, TIMEOUT on request timeout). Nothing to check here.
        if (!info_.use_keep_alive)
            return;

        // DISCONNECTED is terminal — only the CSM may change it.
        if (state_.load(std::memory_order_relaxed) == ControlSignalState::DISCONNECTED)
            return;

        // Keep-alive enabled: transition to TIMEOUT from ANY state (including UNKNOWN)
        // if no heartbeat arrived within 2 × keep_alive_interval_ns.
        // This lets Sources detect that the remote Sink/server has gone away even
        // before a first heartbeat was ever received.
        if (info_.keep_alive_interval_ns > 0)
        {
            const int64_t threshold = 2 * info_.keep_alive_interval_ns;
            if (detail::steadyNs() - lastActivityNs_.load(std::memory_order_relaxed) > threshold)
                state_.store(ControlSignalState::TIMEOUT, std::memory_order_relaxed);
        }
    }

    void _markActivity()
    {
        lastActivityNs_.store(detail::steadyNs(), std::memory_order_relaxed);
        state_.store(ControlSignalState::ACTIVE, std::memory_order_relaxed);
    }

    void _initTransport()
    {
        if (isTopicMode_)
            transport_ = node_->create_publisher<msgT>(info_.channel_name, rclcpp::QoS(10));
        else
        {
            if constexpr (!std::is_void_v<srvT>)
                transport_ = node_->create_client<srvT>(info_.channel_name);
        }

        // Keep-alive subscriber: works in both topic and service modes.
        if (info_.use_keep_alive && info_.keep_alive_interval_ns > 0)
        {
            keepAliveSub_ = node_->create_subscription<std_msgs::msg::String>(
                info_.channel_name + "_keep_alive", rclcpp::QoS(10),
                [this](const std::shared_ptr<std_msgs::msg::String>) { _markActivity(); });
        }
    }

public:
    /**
     * @brief Construct a ControlSignalSource from a ControlSignalInfo descriptor.
     *
     * All behaviour (transport, timeout, keep-alive) is driven by info.
     *
     * Initial state:
     *  - UNKNOWN
     *
     * @param node  Parent ROS 2 node (must outlive this object).
     * @param info  Control signal descriptor.
     */
    ControlSignalSource(rclcpp::Node* node, const msg::ControlSignalInfo& info)
        : node_(node)
        , info_(info)
        , isTopicMode_(std::is_void_v<srvT> ||
                       info.control_signal_mode ==
                           msg::ControlSignalConst::CONTROL_SIGNAL_MODE_TOPIC)
        , transport_(PubPtr{})
        , state_(ControlSignalState::UNKNOWN)
        , lastActivityNs_(detail::steadyNs())
    {
        _initTransport();
    }

    ~ControlSignalSource() override {}

    /**
     * @brief Send a control signal message.
     *
     * Performs a passive timeout check first.
     * Topic mode  : publishes msg; state unchanged (no send-side feedback).
     * Service mode: async call; _markActivity() called on a success response.
     *
     * @param msg  Message to send.
     * @param cmdSuccess  Output parameter set to true if the command was accepted (service mode), always true in topic mode.
     * @return true on successful dispatch; false if transport unavailable.
     */
    bool send(const msgT& msg, bool& cmdSuccess)
    {
        return std::visit([&](auto& t) -> bool
        {
            using T = std::decay_t<decltype(t)>;
            if constexpr (std::is_same_v<T, PubPtr>)
            {
                if (!t) return false;
                t->publish(msg);
                cmdSuccess = true;
                return true;
            }
            else if constexpr (!std::is_same_v<T, std::monostate>)
            {
                if (!t || !t->service_is_ready()) return false;
                auto req  = std::make_shared<typename srvT::Request>();
                req->data = msg;
                auto result = t->async_send_request(req);
                auto future = result.wait_for(std::chrono::nanoseconds(info_.timeout_ns > 0 ? info_.timeout_ns : 50'000'000LL)); // 50 ms default fallback
                if (future == std::future_status::ready)
                {
                    auto res = result.get();
                    _markActivity();
                    cmdSuccess = res->response < rv2_interfaces::msg::ServiceResponseStatusConst::SRV_RES_WARNING;
                    return true;
                }
                else
                {
                    // Timeout waiting for response → mark timeout but don't block the caller.
                    state_.store(ControlSignalState::TIMEOUT, std::memory_order_relaxed);
                    cmdSuccess = false;
                    return false;
                }
            }
            return false;
        }, transport_);
    }

    /** @brief Returns the current state with a passive timeout check. */
    ControlSignalState getState() const override
    {
        _checkTimeout();
        return state_.load(std::memory_order_relaxed);
    }

    void markDisconnected() override
    {
        state_.store(ControlSignalState::DISCONNECTED, std::memory_order_relaxed);
    }

    const msg::ControlSignalInfo& getInfo() const override { return info_; }
};


// ============================================================
//  ControlSignalSink
// ============================================================

/**
 * @brief ControlSignalSink<msgT, srvT> — typed control-signal sink.
 *
 * Transport is a std::variant over Subscription (topic) or Service server,
 * selected once at construction from ControlSignalInfo.
 *
 * State and the last-activity timestamp are held in lock-free atomics;
 * a mutex is used only for latestMsg_.
 * Receiving any message/service-call immediately marks the sink ACTIVE.
 * Timeout is checked passively on read() and getState(), and also in the
 * periodic keep-alive timer callback.
 *
 * Keep-alive (Sink side):
 *   When use_keep_alive == true and keep_alive_interval_ns > 0, a
 *   Publisher<std_msgs::msg::String> on <channel_name>_keep_alive is created
 *   together with a wall timer that fires every keep_alive_interval_ns nanoseconds.
 *   The timer publishes an empty heartbeat and runs _checkTimeout().
 *   Works in both topic and service modes.
 *
 * @tparam msgT  ROS 2 message type (e.g. sensor_msgs::msg::Joy).
 * @tparam srvT  ROS 2 service type.  Request must have a `data` field of type
 *               msgT; Response must have a `success` bool field.
 *               Defaults to void (topic-only).
 */
template<typename msgT, typename srvT = void>
class ControlSignalSink : public BaseControlSignalSink
{
public:
    /**
     * Optional callback invoked immediately after each message is stored.
     * Called with the received message and the Sink's ControlSignalInfo.
     * The callback is invoked from the ROS 2 subscription/service callback thread —
     * it must be thread-safe and must not block for a long time.
     */
    using MsgCb = std::function<void(const msgT&, const msg::ControlSignalInfo&)>;

private:
    rclcpp::Node*          node_;
    msg::ControlSignalInfo info_;

    using SubPtr = typename rclcpp::Subscription<msgT>::SharedPtr;
    using SrvPtr = typename detail::ServicePtrOf<srvT>::type;
    std::variant<SubPtr, SrvPtr> transport_;

    // Keep-alive publisher + periodic timer.
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr keepAlivePub_;
    rclcpp::TimerBase::SharedPtr                        keepAliveTimer_;

    mutable std::mutex   msgMtx_;   // guards latestMsg_ only
    std::optional<msgT>  latestMsg_;

    mutable std::mutex cbMtx_;      // guards msgCb_
    MsgCb              msgCb_;

    mutable std::atomic<ControlSignalState> state_;
    mutable std::atomic<int64_t>            lastActivityNs_;

    // Passive timeout — modifies mutable atomics; safe to call from const context.
    void _checkTimeout() const
    {
        // DISCONNECTED is terminal — nothing to do.
        const ControlSignalState cur = state_.load(std::memory_order_relaxed);
        if (cur == ControlSignalState::DISCONNECTED)
            return;

        if (info_.timeout_ns > 0 &&
            (cur == ControlSignalState::ACTIVE || cur == ControlSignalState::LOW_FREQ))
        {
            const int64_t elapsed = detail::steadyNs() -
                                    lastActivityNs_.load(std::memory_order_relaxed);
            if (elapsed > info_.timeout_ns)
                state_.store(ControlSignalState::TIMEOUT, std::memory_order_relaxed);
            else if (elapsed > info_.timeout_ns / 2)
                state_.store(ControlSignalState::LOW_FREQ, std::memory_order_relaxed);
        }
    }

    // Called on every received message/service-call.
    // Any reception → immediately ACTIVE (from any prior state, including UNKNOWN).
    void _store(const msgT& msg)
    {
        {
            std::lock_guard<std::mutex> lk(msgMtx_);
            latestMsg_ = msg;
        }
        lastActivityNs_.store(detail::steadyNs(), std::memory_order_relaxed);
        state_.store(ControlSignalState::ACTIVE, std::memory_order_relaxed);

        // Fire optional user callback (outside msgMtx_ to avoid re-entrant deadlock).
        MsgCb cb;
        {
            std::lock_guard<std::mutex> lk(cbMtx_);
            cb = msgCb_;
        }
        if (cb) cb(msg, info_);
    }

    // Fired every keep_alive_interval_ns by the wall timer.
    void _keepAliveTimerCb()
    {
        keepAlivePub_->publish(std_msgs::msg::String{});
        _checkTimeout();
    }

    void _initTransport()
    {
        if (info_.use_keep_alive && info_.keep_alive_interval_ns > 0)
        {
            keepAlivePub_ = node_->create_publisher<std_msgs::msg::String>(
                info_.channel_name + "_keep_alive", rclcpp::QoS(10));
            keepAliveTimer_ = node_->create_wall_timer(
                std::chrono::nanoseconds(info_.keep_alive_interval_ns),
                [this]() { _keepAliveTimerCb(); });
        }

        const bool useTopic =
            std::is_void_v<srvT> ||
            (info_.control_signal_mode ==
                 msg::ControlSignalConst::CONTROL_SIGNAL_MODE_TOPIC);

        if (useTopic)
        {
            transport_ = node_->create_subscription<msgT>(
                info_.channel_name, rclcpp::QoS(10),
                [this](const std::shared_ptr<msgT> msg) { _store(*msg); });
        }
        else
        {
            if constexpr (!std::is_void_v<srvT>)
            {
                transport_ = node_->create_service<srvT>(
                    info_.channel_name,
                    [this](const std::shared_ptr<typename srvT::Request>  req,
                                 std::shared_ptr<typename srvT::Response> res) {
                        _store(req->data);
                        res->response = rv2_interfaces::msg::ServiceResponseStatusConst::SRV_RES_SUCCESS;
                    });
            }
        }
    }

public:
    /**
     * @brief Construct a ControlSignalSink from a ControlSignalInfo descriptor.
     *
     * @param node  Parent ROS 2 node (must outlive this object).
     * @param info  Control signal descriptor.
     */
    ControlSignalSink(rclcpp::Node* node, const msg::ControlSignalInfo& info)
        : node_(node)
        , info_(info)
        , transport_(SubPtr{})
        , state_(ControlSignalState::UNKNOWN)
        , lastActivityNs_(detail::steadyNs())
    {
        _initTransport();
    }

    ~ControlSignalSink() override {}

    /**
     * @brief Register (or replace) the per-message callback.
     *
     * The callback is invoked on every stored message, from the ROS 2
     * subscription/service thread.  Pass nullptr to clear the callback.
     *
     * Thread-safe: may be called concurrently with message reception.
     */
    void setMsgCallback(MsgCb cb)
    {
        std::lock_guard<std::mutex> lk(cbMtx_);
        msgCb_ = std::move(cb);
    }

    /**
     * @brief Read the latest received message with a passive timeout check.
     * @param outMsg Output parameter set to the latest message, or default-constructed msgT if none has arrived.
     * @return false if the transport is unavailable or the message is expired by timeout; true otherwise.
     */
    bool read(msgT& outMsg) const
    {
        _checkTimeout();
        std::lock_guard<std::mutex> lk(msgMtx_);
        outMsg = latestMsg_.value_or(msgT{});
        return state_.load(std::memory_order_relaxed) == ControlSignalState::ACTIVE;
    }

    /** @brief Returns the current state with a passive timeout check. */
    ControlSignalState getState() const override
    {
        _checkTimeout();
        return state_.load(std::memory_order_relaxed);
    }

    void markDisconnected() override
    {
        state_.store(ControlSignalState::DISCONNECTED, std::memory_order_relaxed);
    }

    const msg::ControlSignalInfo& getInfo() const override { return info_; }
};


// ============================================================
//  Factory functions
// ============================================================

/**
 * @brief Create a typed ControlSignalSource from a ControlSignalInfo message.
 *
 * Dispatches on info.control_signal_type:
 *  - srvT = void (default): topic-only regardless of info.control_signal_mode.
 *  - srvT provided        : topic or service per info.control_signal_mode.
 *
 * @tparam srvT  Service type; omit (void) for topic-only.
 * @param  node  Parent ROS 2 node.
 * @param  info  Control signal descriptor.
 * @return Shared pointer to BaseControlSignalSource, or nullptr on unknown type.
 */
template<typename srvT = void>
std::shared_ptr<BaseControlSignalSource>
makeControlSignalSource(rclcpp::Node* node, const msg::ControlSignalInfo& info)
{
    const auto& type = info.control_signal_type;
    if (type == msg::ControlSignalConst::CONTROL_SIGNAL_TYPE_JOY)
        return std::make_shared<ControlSignalSource<sensor_msgs::msg::Joy, srvT>>(node, info);
    if (type == msg::ControlSignalConst::CONTROL_SIGNAL_TYPE_TWIST)
        return std::make_shared<ControlSignalSource<geometry_msgs::msg::Twist, srvT>>(node, info);
    if (type == msg::ControlSignalConst::CONTROL_SIGNAL_TYPE_STRING)
        return std::make_shared<ControlSignalSource<std_msgs::msg::String, srvT>>(node, info);
    return nullptr;
}


/**
 * @brief Create a typed ControlSignalSink from a ControlSignalInfo message.
 *
 * Dispatches on info.control_signal_type:
 *  - srvT = void (default): topic-only regardless of info.control_signal_mode.
 *  - srvT provided        : topic or service per info.control_signal_mode.
 *
 * @tparam srvT  Service type; omit (void) for topic-only.
 * @param  node  Parent ROS 2 node.
 * @param  info  Control signal descriptor.
 * @return Shared pointer to BaseControlSignalSink, or nullptr on unknown type.
 */
template<typename srvT = void>
std::shared_ptr<BaseControlSignalSink>
makeControlSignalSink(rclcpp::Node* node, const msg::ControlSignalInfo& info)
{
    const auto& type = info.control_signal_type;
    if (type == msg::ControlSignalConst::CONTROL_SIGNAL_TYPE_JOY)
        return std::make_shared<ControlSignalSink<sensor_msgs::msg::Joy, srvT>>(node, info);
    if (type == msg::ControlSignalConst::CONTROL_SIGNAL_TYPE_TWIST)
        return std::make_shared<ControlSignalSink<geometry_msgs::msg::Twist, srvT>>(node, info);
    if (type == msg::ControlSignalConst::CONTROL_SIGNAL_TYPE_STRING)
        return std::make_shared<ControlSignalSink<std_msgs::msg::String, srvT>>(node, info);
    return nullptr;
}


} // namespace rv2_interfaces
