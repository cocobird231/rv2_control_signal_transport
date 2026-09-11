/**
 * @file control_signal_source.h
 * @brief r1::ControlSignalSource — the sending side of a control signal
 *        (design draft §5), plus the shared entity-status support types
 *        (EntityStatus / EntityDecision / RateRecorder / StateCb) used by
 *        both Source and Sink (§5.2).
 *
 * Single-writer model (D8): the hot path only records; every state
 * transition happens in the CSM status tick through the private friend
 * channel (_calcStatus / _trySealLocalTerminal / _applyStatus).
 */

#ifndef RV2_CONTROL_SIGNAL_TRANSPORT_R1_CONTROL_SIGNAL_SOURCE_H
#define RV2_CONTROL_SIGNAL_TRANSPORT_R1_CONTROL_SIGNAL_SOURCE_H

#include <array>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <shared_mutex>
#include <string>
#include <typeindex>
#include <variant>

#include <rclcpp/rclcpp.hpp>

#include "rv2_control_signal_transport/r1/control_signal_info.h"
#include "rv2_control_signal_transport/r1/liveness_state.h"

namespace rv2_interfaces
{
namespace r1
{

class ControlSignalManager;
class ControlSignalFactory;
struct ManagerTestAccess;

inline int64_t steadyNowNs()
{
    return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch())
        .count();
}

/// Terminal lifecycle routing (§5.2): why a DISCONNECTED decision was made.
enum class LivenessCause : uint8_t
{
    NONE,
    INACTIVITY,
    RESPONSE_FAILURE
};

struct EntityStatus
{
    ControlSignalState state;
    float rateHz;
};

/// Internal decision returned by _calcStatus(); the CSM table keeps only
/// status — the guard tokens are never public (§5.2).
struct EntityDecision
{
    EntityStatus status;
    uint64_t observedActivityGeneration;  // terminal seal validation
    uint64_t observedFailureEpoch;  // response-failure terminal validation
    LivenessCause cause;  // terminal lifecycle routing
};

namespace detail
{

/// Rolling-window call/receive recorder (§5.2). Fixed 8-bucket ring, bucket
/// span = windowNs / 8. Each bucket packs {absoluteBucketNumber, count} into
/// one atomic so epoch and count can never tear; record() CAS-updates the
/// current bucket, calcHz() reads only buckets still inside the window.
/// Lock-free; shared by Source (send record) and Sink (receive record).
class RateRecorder
{
public:
    explicit RateRecorder(int64_t windowNs)  // from ManagerOptions.rateWindowNs
        :
        windowNs_(windowNs)
    {
        for (auto& b : buckets_)
            b.store(0, std::memory_order_relaxed);
    }

    void record(int64_t nowNs)  // hot path, O(1)
    {
        const uint64_t bucketNum = static_cast<uint64_t>(nowNs / (windowNs_ / kBuckets));
        auto& slot = buckets_[bucketNum % kBuckets];
        uint64_t cur = slot.load(std::memory_order_relaxed);
        for (;;)
        {
            const uint64_t curNum = cur >> kCountBits;
            uint64_t next;
            if (curNum == bucketNum)
                next = cur + 1;  // same bucket: ++count
            else if (curNum < bucketNum)
                next = (bucketNum << kCountBits) | 1;  // stale slot: restart
            else
                return;  // newer bucket won: drop
            if (slot.compare_exchange_weak(cur, next, std::memory_order_relaxed))
                return;
        }
    }

    float calcHz(int64_t nowNs) const  // cold path, read-only snapshot
    {
        const int64_t bucketNs = windowNs_ / kBuckets;
        const uint64_t curNum = static_cast<uint64_t>(nowNs / bucketNs);
        uint64_t total = 0;
        for (const auto& b : buckets_)
        {
            const uint64_t v = b.load(std::memory_order_relaxed);
            const uint64_t num = v >> kCountBits;
            // Inside the window = one of the kBuckets most recent spans.
            if (num + kBuckets > curNum && num <= curNum)
                total += v & kCountMask;
        }
        // Normalize over the actually covered span: kBuckets-1 full buckets
        // plus the elapsed fraction of the current one — a steady rate then
        // reads true instead of being diluted by the partial bucket.
        const float fracNs = static_cast<float>(nowNs % bucketNs);
        const float coveredNs = static_cast<float>(bucketNs) * (kBuckets - 1) + fracNs;
        if (coveredNs <= 0.f)
            return 0.f;
        return static_cast<float>(total) / (coveredNs / 1e9f);
    }

private:
    static constexpr unsigned kBuckets = 8;
    static constexpr unsigned kCountBits = 20;  // ~1M records per bucket
    static constexpr uint64_t kCountMask = (uint64_t{1} << kCountBits) - 1;

    std::array<std::atomic<uint64_t>, kBuckets> buckets_;  // packed number+count
    const int64_t windowNs_;
};

/// Transport pointer traits: service mode with srvT = void degenerates to
/// std::monostate (rv2 heritage).
template <typename srvT> struct ClientPtrOf
{
    using type = typename rclcpp::Client<srvT>::SharedPtr;
};
template <> struct ClientPtrOf<void>
{
    using type = std::monostate;
};

}  // namespace detail

/// SendResult replaces rv2's (bool return + bool& cmdSuccess) dual output.
enum class SendResult : uint8_t
{
    OK,  // published (topic) / accepted (service)
    REJECTED,  // service peer answered non-SUCCESS
    NO_TRANSPORT,  // transport unavailable / already shut down
    TIMEOUT,  // service response timed out
    DISCONNECTED,  // endpoint terminally sealed / locally deregistered
    RETRYING,  // logical intent alive, endpoint absent, CSM retrying
    INVALID_CONTEXT  // service send without a serviceable executor / from callback
};

/// Per-state transition callback signature (§5.2).
using StateCb =
    std::function<void(const std::string& controllerName, ControlSignalState oldState, ControlSignalState newState)>;

class BaseControlSignalSource
{
public:
    virtual ~BaseControlSignalSource() = default;
    virtual ControlSignalState getState() const = 0;  // last tick result only
    virtual const ControlSignalInfo& getInfo() const = 0;
    virtual std::type_index msgType() const = 0;
    virtual SendResult sendErased(const void* msg) = 0;
    virtual void shutdown() = 0;  // release rclcpp entities; idempotent

protected:
    BaseControlSignalSource() = default;

private:
    friend class ControlSignalManager;
    friend struct ManagerTestAccess;
    virtual EntityDecision _calcStatus(int64_t nowNs) const = 0;
    virtual bool _trySealLocalTerminal(const EntityDecision& decision) = 0;
    virtual void _sealTerminal() = 0;  // forced / matching lifecycle command
    virtual void _applyStatus(const EntityDecision& decision) = 0;
    /// Manager-side fan-out of the global per-state callback registration
    /// (§8.2 registerSourceStateCallback) onto the endpoint's slot array.
    virtual void _setStateCallbackErased(ControlSignalState s, StateCb cb) = 0;
};

template <typename msgT, typename srvT = void>
class ControlSignalSource : public BaseControlSignalSource,
                            public std::enable_shared_from_this<ControlSignalSource<msgT, srvT>>
{
    friend class ControlSignalManager;
    friend class ControlSignalFactory;  // creator lambda
    friend struct ManagerTestAccess;  // test channel (§5.4; test target only)

public:
    ~ControlSignalSource() override { shutdown(); }

    /// send (§5.3). The call itself is the activity (v1.1.0 self-driven
    /// semantics); the hot path records only — state moves on the next tick.
    SendResult send(const msgT& msg)
    {
        if (shutdown_.load(std::memory_order_acquire))
            return SendResult::NO_TRANSPORT;

        const int64_t now = steadyNowNs();
        if (!liveness_.recordActivity(now))
            return SendResult::DISCONNECTED;  // sealed: must not touch transport
        rate_.record(now);

        if (info_.mode == ControlSignalInfo::MODE_SERVICE)
        {
            if constexpr (std::is_void_v<srvT>)
                return SendResult::NO_TRANSPORT;  // topic-only type (§7)
            else
                return _sendService(msg);
        }

        PubPtr pub;
        {
            std::lock_guard<std::mutex> lk(transportMtx_);
            if (auto* p = std::get_if<PubPtr>(&transport_))
                pub = *p;
        }
        if (!pub)
            return SendResult::NO_TRANSPORT;
        pub->publish(msg);
        return SendResult::OK;
    }

    float sendRateHz() const  // cached only, never triggers a computation
    {
        return cachedRateHz_.load(std::memory_order_relaxed);
    }

    EntityStatus getStatus() const  // {state, cachedRate} combined query
    {
        return {liveness_.state(), cachedRateHz_.load(std::memory_order_relaxed)};
    }

    ControlSignalState getState() const override
    {
        return liveness_.state();  // advanced by the CSM tick (§8.3)
    }

    const ControlSignalInfo& getInfo() const override { return info_; }

    std::type_index msgType() const override { return typeid(msgT); }

    SendResult sendErased(const void* msg) override { return send(*static_cast<const msgT*>(msg)); }

    /// Install / replace the per-state transition callback slot; nullptr
    /// clears. Fired from the tick thread only (§5.3).
    void setStateCallback(ControlSignalState forState, StateCb cb)
    {
        std::unique_lock<std::shared_mutex> lk(stateCbMtx_);
        stateCbs_[static_cast<size_t>(forState)] = std::move(cb);
    }

private:
    void _setStateCallbackErased(ControlSignalState s, StateCb cb) override { setStateCallback(s, std::move(cb)); }

public:
    /// Idempotent: sets the flag, then resets transport under the same lock
    /// used by the send snapshot, so no data race with in-flight sends. An
    /// operation that already linearized (holds its snapshot) may finish;
    /// afterwards every send returns NO_TRANSPORT (§5.3).
    void shutdown() override
    {
        shutdown_.store(true, std::memory_order_release);
        std::lock_guard<std::mutex> lk(transportMtx_);
        transport_ = TransportVariant{};  // resets to a null PubPtr
    }

private:
    using PubPtr = typename rclcpp::Publisher<msgT>::SharedPtr;
    using CliPtr = typename detail::ClientPtrOf<srvT>::type;
    // Doc shape (§5.2): variant<PubPtr, CliPtr>; ClientPtrOf<void> =
    // monostate. "No transport" = the held smart pointer is null.
    using TransportVariant = std::variant<PubPtr, CliPtr>;

    /// private! Only the Manager, the Factory creator and ManagerTestAccess
    /// construct instances (§5.1). rateWindowNs comes from
    /// ManagerOptions.rateWindowNs (default 1 s).
    ControlSignalSource(rclcpp::Node* node, const ControlSignalInfo& info, int64_t rateWindowNs = 1'000'000'000) :
        node_(node),
        info_(info),
        liveness_(steadyNowNs()),
        rate_(rateWindowNs)
    {
        if (info_.mode == ControlSignalInfo::MODE_TOPIC)
        {
            transport_ = node_->create_publisher<msgT>(info_.channel_name, rclcpp::QoS(10));
        }
        else if constexpr (!std::is_void_v<srvT>)
        {
            transport_ = node_->create_client<srvT>(info_.channel_name);
        }
    }

    template <typename T = srvT> SendResult _sendService(const msgT& msg)
    {
        static_assert(!std::is_void_v<T>, "service mode requires a srv type");
        const uint64_t seq = nextRequestSequence_.fetch_add(1, std::memory_order_relaxed);

        typename rclcpp::Client<T>::SharedPtr client;
        {
            std::lock_guard<std::mutex> lk(transportMtx_);
            if (auto* c = std::get_if<CliPtr>(&transport_))
                client = *c;
        }
        if (!client || !client->service_is_ready())
        {
            _recordOutcomeFailure(seq, steadyNowNs());
            return SendResult::NO_TRANSPORT;
        }

        auto request = std::make_shared<typename T::Request>();
        request->data = msg;
        auto future = client->async_send_request(request);

        // No hidden fallback: timeout_ns is mandatory in service mode (§3.2
        // rule 5) and used directly (§5.3).
        if (future.wait_for(std::chrono::nanoseconds(info_.timeout_ns)) != std::future_status::ready)
        {
            client->remove_pending_request(future);  // rv2 audit: pending leak
            _recordOutcomeFailure(seq, steadyNowNs());
            return SendResult::TIMEOUT;
        }

        const auto response = future.get();
        // Any response — including a business-level REJECTED — proves the
        // transport is alive: clear the streak first, then record activity.
        _recordOutcomeSuccess(seq);
        if (!liveness_.recordActivity(steadyNowNs()))
            return SendResult::DISCONNECTED;  // seal won meanwhile
        return response->response == T::Response::SRV_RES_SUCCESS ? SendResult::OK : SendResult::REJECTED;
    }

    /// Only an outcome newer than latestOutcomeRequest may update the streak,
    /// so out-of-order completions of concurrent requests cannot let an older
    /// result overwrite a newer one (§5.3).
    void _recordOutcomeFailure(uint64_t seq, int64_t nowNs)
    {
        std::lock_guard<std::mutex> lk(responseMtx_);
        if (seq <= responseHealth_.latestOutcomeRequest)
            return;
        responseHealth_.latestOutcomeRequest = seq;
        if (!responseHealth_.failing)
        {
            responseHealth_.failing = true;
            responseHealth_.failureSinceNs = nowNs;
            ++responseHealth_.failureEpoch;  // healthy -> failure transition only
        }
    }

    void _recordOutcomeSuccess(uint64_t seq)
    {
        std::lock_guard<std::mutex> lk(responseMtx_);
        if (seq <= responseHealth_.latestOutcomeRequest)
            return;
        responseHealth_.latestOutcomeRequest = seq;
        if (responseHealth_.failing)
        {
            responseHealth_.failing = false;
            ++responseHealth_.failureEpoch;  // failure -> healthy transition only
        }
    }

    static constexpr int _severity(ControlSignalState s)
    {
        switch (s)
        {
        case ControlSignalState::DISCONNECTED:
            return 3;
        case ControlSignalState::TIMEOUT:
            return 2;
        case ControlSignalState::ACTIVE:
            return 1;
        case ControlSignalState::INITIAL:
            return 0;
        }
        return 0;
    }

    /// Non-public (D8): called by the CSM status tick (friend). Pure
    /// computation — merges send-cadence liveness with, in service mode, the
    /// response-health sub-verdict by severity; writes nothing (§5.3).
    EntityDecision _calcStatus(int64_t nowNs) const override
    {
        const float hz = rate_.calcHz(nowNs);
        const LivenessDecision base = liveness_.calcState(nowNs, info_.timeout_ns, info_.disconnect_timeout_ns);

        ControlSignalState state = base.state;
        LivenessCause cause =
            state == ControlSignalState::DISCONNECTED ? LivenessCause::INACTIVITY : LivenessCause::NONE;
        uint64_t epoch = 0;

        if (info_.mode == ControlSignalInfo::MODE_SERVICE)
        {
            ResponseHealth rh;
            {
                std::lock_guard<std::mutex> lk(responseMtx_);
                rh = responseHealth_;
            }
            epoch = rh.failureEpoch;
            if (rh.failing)
            {
                // A live failure streak is immediately TIMEOUT; it deepens to
                // DISCONNECTED only past the disconnect threshold (0 = that
                // layer disabled, variant B — TIMEOUT is then its maximum).
                ControlSignalState rhState = ControlSignalState::TIMEOUT;
                if (info_.disconnect_timeout_ns > 0 && nowNs - rh.failureSinceNs > info_.disconnect_timeout_ns)
                    rhState = ControlSignalState::DISCONNECTED;

                if (_severity(rhState) > _severity(state))
                    state = rhState;
                // Both terminal -> RESPONSE_FAILURE wins, preserving the
                // remote-failure retry semantics of established intents.
                if (rhState == ControlSignalState::DISCONNECTED)
                    cause = LivenessCause::RESPONSE_FAILURE;
            }
        }

        if (state != ControlSignalState::DISCONNECTED)
            cause = LivenessCause::NONE;
        return {{state, hz}, base.observedActivityGeneration, epoch, cause};
    }

    /// INACTIVITY validates via the activity generation; RESPONSE_FAILURE
    /// validates the failure epoch under responseMtx_ and seals inside the
    /// same critical section, giving "success response" vs "death confirmed"
    /// a single linearization order (§5.3).
    bool _trySealLocalTerminal(const EntityDecision& decision) override
    {
        if (decision.cause == LivenessCause::INACTIVITY)
            return liveness_.trySealActivity(decision.observedActivityGeneration);
        if (decision.cause == LivenessCause::RESPONSE_FAILURE)
        {
            std::lock_guard<std::mutex> lk(responseMtx_);
            if (!responseHealth_.failing || responseHealth_.failureEpoch != decision.observedFailureEpoch)
                return false;
            liveness_.sealActivity();
            return true;
        }
        return false;
    }

    void _sealTerminal() override { liveness_.sealActivity(); }

    /// Non-public (D8): writes the cached rate and the state; on old != new
    /// copies the slot under a shared lock and fires it lock-free on the tick
    /// thread (§5.3).
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

    rclcpp::Node* node_;
    ControlSignalInfo info_;
    mutable std::mutex transportMtx_;  // guards transport_ snapshot / reset only
    TransportVariant transport_;
    LivenessState liveness_;
    std::atomic<bool> shutdown_{false};

    detail::RateRecorder rate_;  // send-call rolling window (§5.2)
    std::atomic<float> cachedRateHz_{0.f};  // written by _applyStatus()

    /// Service response-health record (§5.2); the short lock is negligible
    /// next to a service call's cost.
    struct ResponseHealth
    {
        uint64_t latestOutcomeRequest{0};
        bool failing{false};
        int64_t failureSinceNs{0};  // streak start while failing == true
        uint64_t failureEpoch{0};  // bumps only on streak start / clear
    };
    std::atomic<uint64_t> nextRequestSequence_{1};
    mutable std::mutex responseMtx_;
    ResponseHealth responseHealth_;

    std::array<StateCb, 4> stateCbs_;  // per-state slots (§5.2)
    mutable std::shared_mutex stateCbMtx_;
};

}  // namespace r1
}  // namespace rv2_interfaces

#endif  // RV2_CONTROL_SIGNAL_TRANSPORT_R1_CONTROL_SIGNAL_SOURCE_H
