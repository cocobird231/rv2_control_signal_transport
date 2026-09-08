/**
 * @file control_signal_sink.h
 * @brief r1::ControlSignalSink — the receiving side of a control signal
 *        (design draft §6). Zero timers; the receive path only records
 *        (D8) and every state transition happens in the CSM tick.
 */

#ifndef RV2_CONTROL_SIGNAL_TRANSPORT_R1_CONTROL_SIGNAL_SINK_H
#define RV2_CONTROL_SIGNAL_TRANSPORT_R1_CONTROL_SIGNAL_SINK_H

#include <array>
#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <shared_mutex>
#include <thread>
#include <typeindex>
#include <variant>

#include <rclcpp/rclcpp.hpp>

#include "rv2_control_signal_transport/r1/control_signal_info.h"
#include "rv2_control_signal_transport/r1/control_signal_source.h"
#include "rv2_control_signal_transport/r1/liveness_state.h"

namespace rv2_interfaces
{
namespace r1
{

namespace detail
{
template<typename srvT>
struct ServicePtrOf { using type = typename rclcpp::Service<srvT>::SharedPtr; };
template<>
struct ServicePtrOf<void> { using type = std::monostate; };
} // namespace detail

class BaseControlSignalSink
{
public:
    using ErasedMsgCb = std::function<void(const void*, const ControlSignalInfo&)>;
    virtual ~BaseControlSignalSink() = default;
    virtual ControlSignalState getState() const = 0;
    virtual const ControlSignalInfo& getInfo() const = 0;
    virtual std::type_index msgType() const = 0;
    virtual bool readErased(void* outMsg) const = 0;   // true only while ACTIVE
    virtual void setErasedMsgCallback(ErasedMsgCb cb) = 0;
    virtual void shutdown() = 0;

protected:
    BaseControlSignalSink() = default;

private:
    friend class ControlSignalManager;
    friend struct ManagerTestAccess;
    virtual EntityDecision _calcStatus(int64_t nowNs) const = 0;
    virtual bool _trySealLocalTerminal(const EntityDecision& decision) = 0;
    virtual void _sealTerminal() = 0;
    virtual void _applyStatus(const EntityDecision& decision) = 0;
};

template<typename msgT, typename srvT = void>
class ControlSignalSink
    : public BaseControlSignalSink,
      public std::enable_shared_from_this<ControlSignalSink<msgT, srvT>>
{
    friend class ControlSignalManager;
    friend class ControlSignalFactory;
    friend struct ManagerTestAccess;     // test channel (§5.4)

public:
    using MsgCb = std::function<void(const msgT&, const InfoT&)>;

    ~ControlSignalSink() override
    {
        shutdown();
        // No waiter may still block on msgCv_ when it is destroyed: spin
        // until every waitForMessage() has left (§6.3, v1.2.1).
        while (waiters_.load(std::memory_order_acquire) != 0)
            std::this_thread::yield();
    }

    /// read (§6.3): true only while ACTIVE (tick granularity, §4.2 — after
    /// the first message and before the next tick this still returns false;
    /// use waitForMessage()/the msg callback for immediate feedback).
    bool read(msgT& out) const
    {
        if (liveness_.state() != ControlSignalState::ACTIVE)
            return false;
        std::lock_guard<std::mutex> lk(msgMtx_);
        if (!latestMsg_)
            return false;
        out = *latestMsg_;
        return true;
    }

    /// Blocks for the next message arriving AFTER this call (§6.3):
    /// a sequence-number predicate makes it immune to spurious wakeups and
    /// lost wakeups, and independent of state granularity.
    /// timeoutNs 0 = wait forever; message -> true + out; timeout / shutdown
    /// -> false. Blocking call: never use inside a ROS callback (§2.6).
    bool waitForMessage(msgT& out, int64_t timeoutNs = 0) const
    {
        waiters_.fetch_add(1, std::memory_order_acq_rel);
        const uint64_t seq0 = msgSeq_.load(std::memory_order_acquire);
        bool ok = false;
        {
            std::unique_lock<std::mutex> lk(msgMtx_);
            const auto pred = [&] {
                return msgSeq_.load(std::memory_order_relaxed) > seq0 ||
                       shutdown_.load(std::memory_order_relaxed);
            };
            if (timeoutNs <= 0)
                msgCv_.wait(lk, pred);
            else
                msgCv_.wait_for(lk, std::chrono::nanoseconds(timeoutNs), pred);
            if (msgSeq_.load(std::memory_order_relaxed) > seq0 &&
                !shutdown_.load(std::memory_order_relaxed) && latestMsg_)
            {
                out = *latestMsg_;
                ok = true;
            }
        }
        waiters_.fetch_sub(1, std::memory_order_acq_rel);
        return ok;
    }

    float dataRateHz() const   // cached only, never triggers a computation
    {
        return cachedRateHz_.load(std::memory_order_relaxed);
    }

    EntityStatus getStatus() const   // {state, cachedRate} combined query
    {
        return {liveness_.state(), cachedRateHz_.load(std::memory_order_relaxed)};
    }

    /// Install / replace / clear (nullptr) the per-message callback.
    void setMsgCallback(MsgCb cb)
    {
        std::lock_guard<std::mutex> lk(cbMtx_);
        msgCb_ = std::move(cb);
    }

    ControlSignalState getState() const override { return liveness_.state(); }
    const ControlSignalInfo& getInfo() const override { return info_; }
    std::type_index msgType() const override { return typeid(msgT); }

    bool readErased(void* outMsg) const override
    {
        return read(*static_cast<msgT*>(outMsg));
    }

    void setErasedMsgCallback(ErasedMsgCb cb) override
    {
        if (!cb)
        {
            setMsgCallback(nullptr);
            return;
        }
        const ControlSignalInfo& info = info_;
        setMsgCallback([cb = std::move(cb), &info](const msgT& m, const InfoT&) {
            cb(&m, info);
        });
    }

    /// Install / replace the per-state transition callback slot; nullptr
    /// clears. Fired from the tick thread only (§6.3).
    void setStateCallback(ControlSignalState forState, StateCb cb)
    {
        std::unique_lock<std::shared_mutex> lk(stateCbMtx_);
        stateCbs_[static_cast<size_t>(forState)] = std::move(cb);
    }

    /// Idempotent. Wake-up protocol (§6.3, v1.2.1 lost-wakeup fix): the
    /// shutdown_ flag is set under msgMtx_ so it cannot slip between a
    /// waiter's predicate check and its sleep; only then notify_all. The
    /// transport is reset afterwards; a callback already past its seal-side
    /// recordActivity may finish, new ones are rejected.
    void shutdown() override
    {
        {
            std::lock_guard<std::mutex> lk(msgMtx_);
            shutdown_.store(true, std::memory_order_release);
        }
        msgCv_.notify_all();
        std::lock_guard<std::mutex> lk(transportMtx_);
        transport_ = TransportVariant{};
    }

private:
    using SubPtr = typename rclcpp::Subscription<msgT>::SharedPtr;
    using SrvPtr = typename detail::ServicePtrOf<srvT>::type;
    using TransportVariant = std::variant<SubPtr, SrvPtr>;

    /// private! Manager / Factory creator / ManagerTestAccess only (§6.1).
    /// The transport is bound in _bindTransport() right after make_shared —
    /// enable_shared_from_this is unusable inside the constructor and the
    /// receive lambda must capture weak_ptr only, never this (§6.3).
    ControlSignalSink(rclcpp::Node* node, const ControlSignalInfo& info,
                      int64_t rateWindowNs = 1'000'000'000)
        : node_(node)
        , info_(info)
        , liveness_(steadyNowNs())
        , rate_(rateWindowNs)
    {}

    void _bindTransport()
    {
        auto weak = std::weak_ptr<ControlSignalSink>(this->shared_from_this());
        std::lock_guard<std::mutex> lk(transportMtx_);
        if (info_.mode == ControlSignalInfo::MODE_TOPIC)
        {
            transport_ = node_->create_subscription<msgT>(
                info_.channel_name, rclcpp::QoS(10),
                [weak](const msgT& m) {
                    if (auto self = weak.lock())   // lock failure: plain return
                        self->_store(m);
                });
        }
        else if constexpr (!std::is_void_v<srvT>)
        {
            transport_ = node_->create_service<srvT>(
                info_.channel_name,
                [weak](const std::shared_ptr<typename srvT::Request> req,
                       std::shared_ptr<typename srvT::Response> res) {
                    // Store first, then answer SRV_RES_SUCCESS (§6.3) —
                    // like topic mode this only records.
                    if (auto self = weak.lock())
                        self->_store(req->data);
                    res->response = srvT::Response::SRV_RES_SUCCESS;
                });
        }
    }

    /// Receive path (§6.3, fixed order; D8: records only, never state):
    /// 1. recordActivity — false means the terminal seal is established:
    ///    no storage write, no wakeup, no user callback (the linearization
    ///    boundary between data callbacks and terminal deregistration).
    /// 2. store latestMsg_ + ++msgSeq_ under msgMtx_, then notify_all.
    /// 3. copy the callback under cbMtx_, call it lock-free.
    void _store(const msgT& msg)
    {
        const int64_t now = steadyNowNs();
        if (!liveness_.recordActivity(now))
            return;
        rate_.record(now);

        {
            std::lock_guard<std::mutex> lk(msgMtx_);
            latestMsg_ = msg;
            msgSeq_.fetch_add(1, std::memory_order_release);
        }
        msgCv_.notify_all();

        MsgCb cb;
        {
            std::lock_guard<std::mutex> lk(cbMtx_);
            cb = msgCb_;
        }
        if (cb)
            cb(msg, info_);
    }

    /// Non-public (D8): CSM status tick (friend). Read-only snapshot:
    /// rate_.calcHz() + liveness_.calcState() -> {state, rateHz}.
    EntityDecision _calcStatus(int64_t nowNs) const override
    {
        const float hz = rate_.calcHz(nowNs);
        const LivenessDecision base =
            liveness_.calcState(nowNs, info_.timeout_ns, info_.disconnect_timeout_ns);
        return {{base.state, hz},
                base.observedActivityGeneration,
                0,
                base.state == ControlSignalState::DISCONNECTED
                    ? LivenessCause::INACTIVITY
                    : LivenessCause::NONE};
    }

    bool _trySealLocalTerminal(const EntityDecision& decision) override
    {
        return liveness_.trySealActivity(decision.observedActivityGeneration);
    }

    void _sealTerminal() override { liveness_.sealActivity(); }

    /// Non-public (D8): CSM tick writes back state and cache; fires the
    /// per-state slot on old != new — copied under a shared lock, called
    /// lock-free, always on the tick thread (§6.3).
    void _applyStatus(const EntityDecision& decision) override
    {
        cachedRateHz_.store(decision.status.rateHz, std::memory_order_relaxed);
        const ControlSignalState old = liveness_.applyState(decision.status.state);
        if (old == decision.status.state)
            return;
        StateCb cb;
        {
            std::shared_lock<std::shared_mutex> lk(stateCbMtx_);
            cb = stateCbs_[static_cast<size_t>(decision.status.state)];
        }
        if (cb)
            cb(info_.controller_name, old, decision.status.state);
    }

    rclcpp::Node*                node_;
    ControlSignalInfo            info_;
    mutable std::mutex           transportMtx_;   // guards transport_ reset only
    TransportVariant             transport_;
    LivenessState                liveness_;
    mutable std::mutex           msgMtx_;         // guards latestMsg_ (+ cv)
    std::optional<msgT>          latestMsg_;
    mutable std::mutex           cbMtx_;          // guards msgCb_ only
    MsgCb                        msgCb_;
    std::atomic<bool>            shutdown_{false};
    detail::RateRecorder         rate_;           // receive rolling window (§6.2)
    std::atomic<float>           cachedRateHz_{0.f};   // written by _applyStatus()
    std::array<StateCb, 4>       stateCbs_;       // per-state slots (§6.2)
    mutable std::shared_mutex    stateCbMtx_;
    std::atomic<uint64_t>        msgSeq_{0};      // waitForMessage sequence
    mutable std::condition_variable msgCv_;       // msgMtx_ is its lock
    mutable std::atomic<uint64_t>   waiters_{0};  // destructor drain (§6.3)
};

} // namespace r1
} // namespace rv2_interfaces

#endif // RV2_CONTROL_SIGNAL_TRANSPORT_R1_CONTROL_SIGNAL_SINK_H
