/**
 * @file test_liveness_state.cpp
 * @brief L1-L18 unit tests for r1::LivenessState (design draft §4.5).
 *
 * Pure logic, no rclcpp. Time is a fake clock: a manually advanced int64
 * injected through the nowNs parameters, so every case is deterministic.
 * L14 additionally runs multi-writer recordActivity() against a single
 * calc/apply thread and is part of the TSan matrix (§11.4).
 */

#include <algorithm>
#include <atomic>
#include <cstdint>
#include <random>
#include <thread>
#include <vector>

#include <gtest/gtest.h>

#include "rv2_control_signal_transport/r1/liveness_state.h"

namespace
{

using rv2_interfaces::r1::ActivitySnapshot;
using rv2_interfaces::r1::ControlSignalState;
using rv2_interfaces::r1::LivenessDecision;
using rv2_interfaces::r1::LivenessState;

constexpr int64_t kMs = 1'000'000;  // ns per millisecond
constexpr int64_t kTimeout = 100 * kMs;  // variant A thresholds
constexpr int64_t kDisconnect = 1000 * kMs;
constexpr int64_t kT0 = 5'000 * kMs;  // arbitrary fake-clock origin

// L1: fresh instance — state() INITIAL, calc INITIAL, generation 0.
TEST(LivenessStateTest, L1_InitialState)
{
    LivenessState ls(kT0);
    EXPECT_EQ(ls.state(), ControlSignalState::INITIAL);

    const LivenessDecision d = ls.calcState(kT0 + kMs, kTimeout, kDisconnect);
    EXPECT_EQ(d.state, ControlSignalState::INITIAL);
    EXPECT_EQ(d.observedActivityGeneration, 0u);
}

// L2: recordActivity then calcState — true, ACTIVE, generation incremented.
TEST(LivenessStateTest, L2_RecordThenActive)
{
    LivenessState ls(kT0);
    EXPECT_TRUE(ls.recordActivity(kT0 + kMs));

    const LivenessDecision d = ls.calcState(kT0 + 2 * kMs, kTimeout, kDisconnect);
    EXPECT_EQ(d.state, ControlSignalState::ACTIVE);
    EXPECT_EQ(d.observedActivityGeneration, 1u);
}

// L3: elapsed > timeout — TIMEOUT.
TEST(LivenessStateTest, L3_TimeoutDecision)
{
    LivenessState ls(kT0);
    ASSERT_TRUE(ls.recordActivity(kT0));

    const LivenessDecision d = ls.calcState(kT0 + kTimeout + 1, kTimeout, kDisconnect);
    EXPECT_EQ(d.state, ControlSignalState::TIMEOUT);
}

// L4: activity resumes after a TIMEOUT verdict — next calc is ACTIVE, no
// special transition condition.
TEST(LivenessStateTest, L4_TimeoutRecovers)
{
    LivenessState ls(kT0);
    ASSERT_TRUE(ls.recordActivity(kT0));
    ASSERT_EQ(ls.calcState(kT0 + kTimeout + 1, kTimeout, kDisconnect).state, ControlSignalState::TIMEOUT);

    const int64_t resume = kT0 + kTimeout + 2;
    ASSERT_TRUE(ls.recordActivity(resume));
    EXPECT_EQ(ls.calcState(resume + kMs, kTimeout, kDisconnect).state, ControlSignalState::ACTIVE);
}

// L5: elapsed > disconnect — DISCONNECTED, including the single-jump case
// where one calc crosses both thresholds at once (disconnect wins).
TEST(LivenessStateTest, L5_DisconnectDecisionSingleJump)
{
    LivenessState ls(kT0);
    ASSERT_TRUE(ls.recordActivity(kT0));

    // Previous calc saw ACTIVE; the next one jumps beyond both thresholds.
    ASSERT_EQ(ls.calcState(kT0 + kMs, kTimeout, kDisconnect).state, ControlSignalState::ACTIVE);
    const LivenessDecision d = ls.calcState(kT0 + kDisconnect + 1, kTimeout, kDisconnect);
    EXPECT_EQ(d.state, ControlSignalState::DISCONNECTED);
}

// L6: never active, elapsed > timeout but <= disconnect — stays INITIAL.
TEST(LivenessStateTest, L6_NeverActiveNoTimeout)
{
    LivenessState ls(kT0);
    const LivenessDecision d = ls.calcState(kT0 + kTimeout + 1, kTimeout, kDisconnect);
    EXPECT_EQ(d.state, ControlSignalState::INITIAL);
    EXPECT_EQ(d.observedActivityGeneration, 0u);
}

// L7: never active, elapsed > disconnect (from construction) — DISCONNECTED.
TEST(LivenessStateTest, L7_NeverActiveDisconnect)
{
    LivenessState ls(kT0);
    const LivenessDecision d = ls.calcState(kT0 + kDisconnect + 1, kTimeout, kDisconnect);
    EXPECT_EQ(d.state, ControlSignalState::DISCONNECTED);
}

// L8: elapsed exactly equal to a threshold does not trigger it (strict >).
TEST(LivenessStateTest, L8_BoundaryIsStrict)
{
    LivenessState ls(kT0);
    ASSERT_TRUE(ls.recordActivity(kT0));

    EXPECT_EQ(ls.calcState(kT0 + kTimeout, kTimeout, kDisconnect).state, ControlSignalState::ACTIVE);
    EXPECT_EQ(ls.calcState(kT0 + kDisconnect, kTimeout, kDisconnect).state,
              ControlSignalState::TIMEOUT);  // > timeout, == disconnect

    // Never-active boundary: elapsed == disconnect stays INITIAL.
    LivenessState fresh(kT0);
    EXPECT_EQ(fresh.calcState(kT0 + kDisconnect, kTimeout, kDisconnect).state, ControlSignalState::INITIAL);
}

// L9: applyState returns the previous state and state() reads the new one.
// (No CAS on the state field by design — single writer, §4.2/D8.)
TEST(LivenessStateTest, L9_ApplyStateReturnsOld)
{
    LivenessState ls(kT0);
    EXPECT_EQ(ls.applyState(ControlSignalState::ACTIVE), ControlSignalState::INITIAL);
    EXPECT_EQ(ls.state(), ControlSignalState::ACTIVE);
    EXPECT_EQ(ls.applyState(ControlSignalState::TIMEOUT), ControlSignalState::ACTIVE);
    EXPECT_EQ(ls.state(), ControlSignalState::TIMEOUT);
}

// L10: calcState is a pure function — repeated calls with the same inputs
// give the same result and no field changes.
TEST(LivenessStateTest, L10_CalcStateIsPure)
{
    LivenessState ls(kT0);
    ASSERT_TRUE(ls.recordActivity(kT0 + kMs));
    const ActivitySnapshot before = ls.activitySnapshot();

    const int64_t now = kT0 + kTimeout + 2;
    const LivenessDecision d1 = ls.calcState(now, kTimeout, kDisconnect);
    const LivenessDecision d2 = ls.calcState(now, kTimeout, kDisconnect);
    EXPECT_EQ(d1.state, d2.state);
    EXPECT_EQ(d1.observedActivityGeneration, d2.observedActivityGeneration);

    const ActivitySnapshot after = ls.activitySnapshot();
    EXPECT_EQ(before.lastActivityNs, after.lastActivityNs);
    EXPECT_EQ(before.generation, after.generation);
    EXPECT_EQ(before.sealed, after.sealed);
    EXPECT_EQ(ls.state(), ControlSignalState::INITIAL);  // never applied
}

// L11: variant B (disconnect = 0) — with huge elapsed an active entity stays
// TIMEOUT and a never-active one stays INITIAL; DISCONNECTED unreachable.
TEST(LivenessStateTest, L11_VariantB_NoDisconnect)
{
    const int64_t huge = kT0 + 400L * 24 * 3600 * 1'000'000'000L;  // ~400 days

    LivenessState active(kT0);
    ASSERT_TRUE(active.recordActivity(kT0));
    EXPECT_EQ(active.calcState(huge, kTimeout, 0).state, ControlSignalState::TIMEOUT);

    LivenessState never(kT0);
    EXPECT_EQ(never.calcState(huge, kTimeout, 0).state, ControlSignalState::INITIAL);
}

// L12: variant C (timeout = 0) — TIMEOUT unreachable; crossing disconnect
// goes straight to DISCONNECTED.
TEST(LivenessStateTest, L12_VariantC_NoTimeout)
{
    LivenessState ls(kT0);
    ASSERT_TRUE(ls.recordActivity(kT0));

    EXPECT_EQ(ls.calcState(kT0 + kDisconnect, 0, kDisconnect).state,
              ControlSignalState::ACTIVE);  // huge elapsed, still ACTIVE
    EXPECT_EQ(ls.calcState(kT0 + kDisconnect + 1, 0, kDisconnect).state, ControlSignalState::DISCONNECTED);
}

// L13: variant D (both 0) — calc never leaves the current phase regardless of
// elapsed; only applyState(DISCONNECTED) can (forced path).
TEST(LivenessStateTest, L13_VariantD_BothDisabled)
{
    const int64_t huge = kT0 + 400L * 24 * 3600 * 1'000'000'000L;

    LivenessState active(kT0);
    ASSERT_TRUE(active.recordActivity(kT0));
    EXPECT_EQ(active.calcState(huge, 0, 0).state, ControlSignalState::ACTIVE);

    LivenessState never(kT0);
    EXPECT_EQ(never.calcState(huge, 0, 0).state, ControlSignalState::INITIAL);

    // Forced exit stays available.
    active.sealActivity();
    EXPECT_EQ(active.applyState(ControlSignalState::DISCONNECTED), ControlSignalState::INITIAL);
    EXPECT_EQ(active.state(), ControlSignalState::DISCONNECTED);
}

// L14: multiple writers record with out-of-order now values — the timestamp
// never regresses, the generation equals the number of successful records,
// and (under TSan) there is no data race.
TEST(LivenessStateTest, L14_MultiWriterOutOfOrder)
{
    constexpr int kThreads = 8;
    constexpr int kPerThread = 2'000;

    LivenessState ls(kT0);

    // Out-of-order timestamps: one shuffled global sequence, deterministic seed.
    std::vector<int64_t> stamps(kThreads * kPerThread);
    for (size_t i = 0; i < stamps.size(); ++i)
        stamps[i] = kT0 + static_cast<int64_t>(i + 1);
    std::mt19937 rng(42);
    std::shuffle(stamps.begin(), stamps.end(), rng);

    std::atomic<int64_t> maxSeen{0};
    std::atomic<uint64_t> accepted{0};
    std::vector<std::thread> writers;
    for (int t = 0; t < kThreads; ++t)
    {
        writers.emplace_back(
            [&, t]()
            {
                for (int i = 0; i < kPerThread; ++i)
                {
                    const int64_t now = stamps[t * kPerThread + i];
                    if (ls.recordActivity(now))
                        accepted.fetch_add(1, std::memory_order_relaxed);
                    int64_t m = maxSeen.load(std::memory_order_relaxed);
                    while (now > m && !maxSeen.compare_exchange_weak(m, now, std::memory_order_relaxed))
                    {
                    }
                }
            });
    }

    // Single tick-role thread (calc/apply, §4.5): non-terminal states may be
    // applied concurrently with the writers; the timestamp never regresses.
    std::atomic<bool> stop{false};
    int64_t lastTs = 0;
    std::thread ticker(
        [&]()
        {
            while (!stop.load(std::memory_order_relaxed))
            {
                const ActivitySnapshot s = ls.activitySnapshot();
                EXPECT_GE(s.lastActivityNs, lastTs);
                lastTs = s.lastActivityNs;
                const auto d = ls.calcState(kT0 + 1, kTimeout, kDisconnect);
                ls.applyState(d.state);
            }
        });

    for (auto& w : writers)
        w.join();
    stop.store(true, std::memory_order_relaxed);
    ticker.join();

    const ActivitySnapshot s = ls.activitySnapshot();
    EXPECT_EQ(s.generation, accepted.load());
    EXPECT_EQ(s.generation, static_cast<uint64_t>(kThreads) * kPerThread);
    EXPECT_EQ(s.lastActivityNs, maxSeen.load());
    EXPECT_FALSE(s.sealed);

    // Single-thread out-of-order tail: older stamp still counts as a record
    // but the timestamp holds.
    const int64_t held = s.lastActivityNs;
    EXPECT_TRUE(ls.recordActivity(held - 5'000));
    EXPECT_EQ(ls.activitySnapshot().lastActivityNs, held);
    EXPECT_EQ(ls.activitySnapshot().generation, s.generation + 1);

    // The activity-generation CAS also competes with the terminal seal. Both
    // start from one observed generation; exactly one may accept its mutation.
    for (int round = 0; round < 64; ++round)
    {
        LivenessState racing(kT0);
        ASSERT_TRUE(racing.recordActivity(kT0));
        const auto terminal = racing.calcState(kT0 + kDisconnect + 1, kTimeout, kDisconnect);
        ASSERT_EQ(terminal.state, ControlSignalState::DISCONNECTED);
        std::atomic<int> ready{0};
        std::atomic<bool> go{false};
        bool recorded = false;
        bool sealed = false;
        std::thread writer(
            [&]()
            {
                ready.fetch_add(1);
                while (!go.load())
                    std::this_thread::yield();
                recorded = racing.recordActivity(kT0 + kDisconnect + 2);
            });
        std::thread sealer(
            [&]()
            {
                ready.fetch_add(1);
                while (!go.load())
                    std::this_thread::yield();
                sealed = racing.trySealActivity(terminal.observedActivityGeneration);
            });
        while (ready.load() != 2)
            std::this_thread::yield();
        go.store(true);
        writer.join();
        sealer.join();
        EXPECT_NE(recorded, sealed);
        EXPECT_EQ(racing.activitySnapshot().generation, recorded ? 2u : 1u);
        EXPECT_EQ(racing.activitySnapshot().sealed, sealed);
        racing.sealActivity();
        EXPECT_FALSE(racing.recordActivity(kT0 + kDisconnect + 3));
    }
}

// L15: activity lands after a TIMEOUT computation — committing the stale
// TIMEOUT this round is allowed (non-destructive) and the next tick returns
// ACTIVE.
TEST(LivenessStateTest, L15_StaleTimeoutCommitRecovers)
{
    LivenessState ls(kT0);
    ASSERT_TRUE(ls.recordActivity(kT0));

    const int64_t tick1 = kT0 + kTimeout + 1;
    const LivenessDecision d = ls.calcState(tick1, kTimeout, kDisconnect);
    ASSERT_EQ(d.state, ControlSignalState::TIMEOUT);

    // Activity slips in between calc and apply.
    ASSERT_TRUE(ls.recordActivity(tick1));

    // Non-destructive commit of the stale verdict is fine.
    EXPECT_EQ(ls.applyState(d.state), ControlSignalState::INITIAL);
    EXPECT_EQ(ls.state(), ControlSignalState::TIMEOUT);

    // Next tick recovers.
    const LivenessDecision d2 = ls.calcState(tick1 + kMs, kTimeout, kDisconnect);
    EXPECT_EQ(d2.state, ControlSignalState::ACTIVE);
    EXPECT_EQ(ls.applyState(d2.state), ControlSignalState::TIMEOUT);
}

// L16: activity lands between a DISCONNECTED computation and the seal — the
// generation mismatch makes the seal fail, nothing may be applied/erased, and
// the next tick sees ACTIVE.
TEST(LivenessStateTest, L16_SealLosesToActivity)
{
    LivenessState ls(kT0);
    ASSERT_TRUE(ls.recordActivity(kT0));

    const int64_t tick1 = kT0 + kDisconnect + 1;
    const LivenessDecision d = ls.calcState(tick1, kTimeout, kDisconnect);
    ASSERT_EQ(d.state, ControlSignalState::DISCONNECTED);

    ASSERT_TRUE(ls.recordActivity(tick1));  // activity wins
    EXPECT_FALSE(ls.trySealActivity(d.observedActivityGeneration));

    // Seal failure changed nothing: not sealed, activity still accepted.
    const ActivitySnapshot s = ls.activitySnapshot();
    EXPECT_FALSE(s.sealed);
    EXPECT_EQ(s.generation, d.observedActivityGeneration + 1);
    EXPECT_TRUE(ls.recordActivity(tick1 + 1));

    // This round must not apply DISCONNECTED; next tick computes ACTIVE.
    EXPECT_EQ(ls.calcState(tick1 + kMs, kTimeout, kDisconnect).state, ControlSignalState::ACTIVE);
    EXPECT_EQ(ls.state(), ControlSignalState::INITIAL);  // untouched
}

// L17: once the seal wins, hot-path activity is rejected and the terminal
// state can be applied exactly once before shutdown/erase.
TEST(LivenessStateTest, L17_SealWinsRejectsActivity)
{
    LivenessState ls(kT0);
    ASSERT_TRUE(ls.recordActivity(kT0));

    const int64_t tick1 = kT0 + kDisconnect + 1;
    const LivenessDecision d = ls.calcState(tick1, kTimeout, kDisconnect);
    ASSERT_EQ(d.state, ControlSignalState::DISCONNECTED);

    ASSERT_TRUE(ls.trySealActivity(d.observedActivityGeneration));

    // Hot path is now rejected; generation is frozen (the timestamp CAS-max
    // may still run before the seal check, harmlessly — §4.4).
    const ActivitySnapshot before = ls.activitySnapshot();
    EXPECT_TRUE(before.sealed);
    EXPECT_FALSE(ls.recordActivity(tick1 + 1));
    const ActivitySnapshot after = ls.activitySnapshot();
    EXPECT_EQ(after.generation, before.generation);

    // A second seal attempt with the same generation fails (already sealed)…
    EXPECT_FALSE(ls.trySealActivity(d.observedActivityGeneration));
    // …while the unconditional seal stays idempotent.
    ls.sealActivity();
    EXPECT_TRUE(ls.activitySnapshot().sealed);

    // Terminal apply: old != new exactly once → caller fires one callback,
    // then shutdown / erase is safe.
    EXPECT_EQ(ls.applyState(ControlSignalState::DISCONNECTED), ControlSignalState::INITIAL);
    EXPECT_EQ(ls.applyState(ControlSignalState::DISCONNECTED),
              ControlSignalState::DISCONNECTED);  // no further old != new edge
}

// L18: first tick after recordActivity is already past timeout — INITIAL may
// apply TIMEOUT directly; a never-active entity at the same elapsed stays
// INITIAL.
TEST(LivenessStateTest, L18_LateFirstTick)
{
    LivenessState recorded(kT0);
    ASSERT_TRUE(recorded.recordActivity(kT0));
    ASSERT_EQ(recorded.state(), ControlSignalState::INITIAL);

    const int64_t lateTick = kT0 + kTimeout + 1;  // < disconnect
    const LivenessDecision d = recorded.calcState(lateTick, kTimeout, kDisconnect);
    EXPECT_EQ(d.state, ControlSignalState::TIMEOUT);
    EXPECT_EQ(recorded.applyState(d.state), ControlSignalState::INITIAL);
    EXPECT_EQ(recorded.state(), ControlSignalState::TIMEOUT);

    LivenessState never(kT0);
    EXPECT_EQ(never.calcState(lateTick, kTimeout, kDisconnect).state, ControlSignalState::INITIAL);
}

}  // namespace

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
