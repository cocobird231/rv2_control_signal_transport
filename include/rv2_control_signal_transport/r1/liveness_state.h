/**
 * @file liveness_state.h
 * @brief r1::LivenessState — pure-logic activity recording and 4-state
 *        liveness decision (design draft §4).
 *
 * Pure C++, no ROS dependency. Since v1.1.0 (D8) the class does exactly three
 * things: atomic hot-path recording, const pure-state computation, and passive
 * state acceptance reserved for the CSM tick (single-writer model). State
 * progression decisions live in the CSM tick (§8.3); the only CAS kept here is
 * the activity-generation terminal seal, which protects activity acceptance
 * and terminal linearization — it never writes state.
 */

#ifndef RV2_CONTROL_SIGNAL_TRANSPORT_R1_LIVENESS_STATE_H
#define RV2_CONTROL_SIGNAL_TRANSPORT_R1_LIVENESS_STATE_H

#include <atomic>
#include <cstdint>

namespace rv2_interfaces
{
namespace r1
{

enum class ControlSignalState : uint8_t
{
    INITIAL,
    ACTIVE,
    TIMEOUT,
    DISCONNECTED
};

struct ActivitySnapshot
{
    int64_t lastActivityNs;
    uint64_t generation;
    bool sealed;
};

struct LivenessDecision
{
    ControlSignalState state;
    uint64_t observedActivityGeneration;
};

class LivenessState
{
public:
    explicit LivenessState(int64_t nowNs) :
        createdNs_(nowNs),
        lastActivityNs_(nowNs),
        activityWord_(0),
        state_(ControlSignalState::INITIAL)
    {
    }

    LivenessState(const LivenessState&) = delete;
    LivenessState& operator=(const LivenessState&) = delete;

    /// Hot path (send() / receive callback): records only, never writes state.
    /// The timestamp is published (atomic max, never regressing) before the
    /// generation increment, so a reader that acquires a new generation always
    /// sees a matching-or-newer timestamp (§4.4).
    /// @return false = the terminal seal is already established; the caller
    ///         must stop this data operation (no transport / message storage).
    bool recordActivity(int64_t nowNs)
    {
        int64_t prev = lastActivityNs_.load(std::memory_order_relaxed);
        while (nowNs > prev && !lastActivityNs_.compare_exchange_weak(
                                   prev, nowNs, std::memory_order_release, std::memory_order_relaxed))
        {
        }

        uint64_t word = activityWord_.load(std::memory_order_relaxed);
        do
        {
            if (word & kSealed)
                return false;
        } while (
            !activityWord_.compare_exchange_weak(word, word + 1, std::memory_order_release, std::memory_order_relaxed));
        return true;
    }

    /// Pure computation (CSM tick only): derives the due state from the
    /// recorded activity and the two thresholds; writes nothing. A threshold
    /// of 0 disables that layer (§2.3.1 variants A-D). The current state is
    /// deliberately not consulted, which makes TIMEOUT recoverable at any
    /// time for free (§4.4).
    ///
    /// Never-active entities (generation == 0) measure elapsed from
    /// construction and have no "interruption" semantics: they stay INITIAL
    /// and only the disconnect threshold applies (§2.3).
    LivenessDecision calcState(int64_t nowNs, int64_t timeoutNs, int64_t disconnectNs) const
    {
        const uint64_t word = activityWord_.load(std::memory_order_acquire);
        const uint64_t generation = word & ~kSealed;

        if (generation == 0)
        {
            const int64_t elapsed = nowNs - createdNs_;
            const bool dead = disconnectNs > 0 && elapsed > disconnectNs;
            return {dead ? ControlSignalState::DISCONNECTED : ControlSignalState::INITIAL, generation};
        }

        const int64_t elapsed = nowNs - lastActivityNs_.load(std::memory_order_relaxed);
        ControlSignalState s = ControlSignalState::ACTIVE;
        if (disconnectNs > 0 && elapsed > disconnectNs)  // strict >, disconnect first
            s = ControlSignalState::DISCONNECTED;
        else if (timeoutNs > 0 && elapsed > timeoutNs)  // strict >
            s = ControlSignalState::TIMEOUT;
        return {s, generation};
    }

    /// Destructive commit gate for the local timeout path. Atomically sets
    /// sealed only while the generation still equals the computed snapshot;
    /// failure means activity was accepted after the computation and this
    /// round must not apply DISCONNECTED / erase (§4.2, L16). After a
    /// successful seal every recordActivity() returns false; the successful
    /// CAS is the linearization point of "death confirmed".
    bool trySealActivity(uint64_t observedGeneration)
    {
        uint64_t expected = observedGeneration;  // implies not sealed
        return activityWord_.compare_exchange_strong(
            expected, observedGeneration | kSealed, std::memory_order_acq_rel, std::memory_order_relaxed);
    }

    /// Forced / matching remote lifecycle removal: unconditionally establishes
    /// the terminal seal. Idempotent.
    void sealActivity() { activityWord_.fetch_or(kSealed, std::memory_order_acq_rel); }

    /// CSM tick only: stores the state and returns the previous one (the
    /// caller fires the state callback on old != new). Unconditional exchange,
    /// no CAS — the single-writer model makes concurrent state writers
    /// structurally impossible (§4.2, D8).
    ControlSignalState applyState(ControlSignalState s) { return state_.exchange(s, std::memory_order_acq_rel); }

    /// Reads the last applied state; never triggers a computation.
    ControlSignalState state() const { return state_.load(std::memory_order_acquire); }

    ActivitySnapshot activitySnapshot() const
    {
        const uint64_t word = activityWord_.load(std::memory_order_acquire);
        return {lastActivityNs_.load(std::memory_order_relaxed), word & ~kSealed, (word & kSealed) != 0};
    }

private:
    static constexpr uint64_t kSealed = uint64_t{1} << 63;

    const int64_t createdNs_;  // construction time, read-only
    std::atomic<int64_t> lastActivityNs_;  // atomic max, never regresses
    std::atomic<uint64_t> activityWord_;  // 1-bit sealed + 63-bit generation
    std::atomic<ControlSignalState> state_;  // last applyState() result
};

}  // namespace r1
}  // namespace rv2_interfaces

#endif  // RV2_CONTROL_SIGNAL_TRANSPORT_R1_LIVENESS_STATE_H
