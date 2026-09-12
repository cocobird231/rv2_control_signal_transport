/**
 * @file test_transport.cpp
 * @brief S1-S16 (suite SourceTest, §5.4) and K1-K16 (suite SinkTest, §6.4).
 *
 * Tick simulation (v1.1.0/D8): state progression is driven through the
 * ManagerTestAccess friend channel (_calcStatus + seal + _applyStatus),
 * equivalent to one CSM tick — no real Manager involved. Timeout cases
 * inject a fake now (real steady now + offset) instead of sleeping.
 */

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>

#include <r1_interfaces/srv/control_signal_joy.hpp>

#include "r1_test_utils.h"

namespace
{

using namespace std::chrono_literals;
using rv2_interfaces::r1::ControlSignalInfo;
using rv2_interfaces::r1::ControlSignalState;
using rv2_interfaces::r1::EntityDecision;
using rv2_interfaces::r1::LivenessCause;
using rv2_interfaces::r1::makeTransportInfo;
using rv2_interfaces::r1::ManagerTestAccess;
using rv2_interfaces::r1::SendResult;
using rv2_interfaces::r1::steadyNowNs;
using Joy = sensor_msgs::msg::Joy;
using JoySrv = r1_interfaces::srv::ControlSignalJoy;

constexpr int64_t kMs = 1'000'000;
constexpr int64_t kTimeout = 150 * kMs;
constexpr int64_t kDisconnect = 600 * kMs;

Joy makeJoy(float axis0)
{
    Joy j;
    j.axes = {axis0};
    return j;
}

ControlSignalInfo topicInfo(const std::string& channel)
{
    return makeTransportInfo(
        channel, ControlSignalInfo::MODE_TOPIC, ControlSignalInfo::TYPE_JOY, kTimeout, kDisconnect);
}

ControlSignalInfo serviceInfo(const std::string& channel)
{
    return makeTransportInfo(
        channel, ControlSignalInfo::MODE_SERVICE, ControlSignalInfo::TYPE_JOY, kTimeout, kDisconnect);
}

class SourceTest : public rv2_interfaces::r1::CsmTestBase
{
protected:
    /// Serving mock: answers every request on the shared executor.
    rclcpp::Service<JoySrv>::SharedPtr
    makeServer(const std::string& channel, int8_t response, std::atomic<int>* hits = nullptr)
    {
        return node_->create_service<JoySrv>(
            channel,
            [response, hits](const std::shared_ptr<JoySrv::Request>, std::shared_ptr<JoySrv::Response> res)
            {
                if (hits)
                    hits->fetch_add(1);
                res->response = response;
            });
    }
};

class SinkTest : public rv2_interfaces::r1::CsmTestBase
{
};

// ───────────────────────── SourceTest S1-S16 ─────────────────────────

// S1: topic mode initial — INITIAL; send() == OK.
TEST_F(SourceTest, S1_TopicInitial)
{
    auto src = ManagerTestAccess::createSource<Joy>(node_.get(), topicInfo("s1/joy"));
    EXPECT_EQ(src->getState(), ControlSignalState::INITIAL);
    EXPECT_EQ(src->send(makeJoy(1.f)), SendResult::OK);
}

// S2: topic send then simulated tick — _calcStatus ACTIVE (send is the
// activity, v1.1.0); after apply getState() follows.
TEST_F(SourceTest, S2_TopicSendThenTick)
{
    auto src = ManagerTestAccess::createSource<Joy>(node_.get(), topicInfo("s2/joy"));
    ASSERT_EQ(src->send(makeJoy(1.f)), SendResult::OK);

    const auto d = ManagerTestAccess::calc(*src, steadyNowNs());
    EXPECT_EQ(d.status.state, ControlSignalState::ACTIVE);
    ManagerTestAccess::apply(*src, d);
    EXPECT_EQ(src->getState(), ControlSignalState::ACTIVE);
}

// S3: service mode send success — OK immediately; tick -> ACTIVE.
TEST_F(SourceTest, S3_ServiceSendSuccess)
{
    auto server = makeServer("s3/joy", JoySrv::Response::SRV_RES_SUCCESS);
    auto src = ManagerTestAccess::createSource<Joy, JoySrv>(node_.get(), serviceInfo("s3/joy"));
    // Guard against discovery latency: the row expects OK, not a spurious
    // NO_TRANSPORT from a not-yet-discovered server.
    auto probe = node_->create_client<JoySrv>("s3/joy");
    ASSERT_TRUE(probe->wait_for_service(3s));
    EXPECT_EQ(src->send(makeJoy(1.f)), SendResult::OK);

    const auto d = ManagerTestAccess::calc(*src, steadyNowNs());
    EXPECT_EQ(d.status.state, ControlSignalState::ACTIVE);
    ManagerTestAccess::apply(*src, d);
    EXPECT_EQ(src->getState(), ControlSignalState::ACTIVE);
}

// S4: service server rejects — REJECTED; a response is still activity
// (tick -> ACTIVE).
TEST_F(SourceTest, S4_ServiceRejectedStillActivity)
{
    auto server = makeServer("s4/joy", JoySrv::Response::SRV_RES_REJECTED);
    auto src = ManagerTestAccess::createSource<Joy, JoySrv>(node_.get(), serviceInfo("s4/joy"));
    auto probe = node_->create_client<JoySrv>("s4/joy");
    ASSERT_TRUE(probe->wait_for_service(3s));
    EXPECT_EQ(src->send(makeJoy(1.f)), SendResult::REJECTED);

    const auto d = ManagerTestAccess::calc(*src, steadyNowNs());
    EXPECT_EQ(d.status.state, ControlSignalState::ACTIVE);
}

// S5: no server -> NO_TRANSPORT; response timeout -> TIMEOUT. The failure
// streak drives the next tick to TIMEOUT even under continued high-frequency
// sending, and past the disconnect threshold the decision is DISCONNECTED.
TEST_F(SourceTest, S5_ServiceFailureStreak)
{
    // (a) no server at all -> NO_TRANSPORT.
    auto src = ManagerTestAccess::createSource<Joy, JoySrv>(node_.get(), serviceInfo("s5/joy"));
    EXPECT_EQ(src->send(makeJoy(1.f)), SendResult::NO_TRANSPORT);

    // (b) discovered but never served (server node not spun) -> TIMEOUT.
    // Long disconnect threshold: discovery polling and the per-send response
    // waits accumulate real time that must not cross it prematurely.
    const int64_t longDisconnect = 10'000 * kMs;
    const auto infoB = rv2_interfaces::r1::makeTransportInfo(
        "s5b/joy", ControlSignalInfo::MODE_SERVICE, ControlSignalInfo::TYPE_JOY, kTimeout, longDisconnect);
    auto deadNode = std::make_shared<rclcpp::Node>("s5_dead_server");
    auto deadServer =
        deadNode->create_service<JoySrv>("s5b/joy",
                                         [](const std::shared_ptr<JoySrv::Request>, std::shared_ptr<JoySrv::Response>)
                                         {
                                         });
    auto src2 = ManagerTestAccess::createSource<Joy, JoySrv>(node_.get(), infoB);
    // Wait for discovery, then send: the request is never processed.
    for (int i = 0; i < 100 && src2->send(makeJoy(1.f)) == SendResult::NO_TRANSPORT; ++i)
        std::this_thread::sleep_for(20ms);
    EXPECT_EQ(src2->send(makeJoy(1.f)), SendResult::TIMEOUT);

    // High-frequency sends keep the liveness timestamp fresh, but the streak
    // must not be masked. The tick is computed at a fake now only 10 ms after
    // the last send's activity record, where the send-cadence base alone is
    // still ACTIVE — only the failure streak can force TIMEOUT here.
    for (int i = 0; i < 2; ++i)
        EXPECT_EQ(src2->send(makeJoy(1.f)), SendResult::TIMEOUT);
    const int64_t tEntry = steadyNowNs();  // last send records activity ~here
    EXPECT_EQ(src2->send(makeJoy(1.f)), SendResult::TIMEOUT);
    auto d = ManagerTestAccess::calc(*src2, tEntry + 10 * kMs);
    EXPECT_EQ(d.status.state, ControlSignalState::TIMEOUT);

    // Past the disconnect threshold (fake now) -> DISCONNECTED,
    // cause = RESPONSE_FAILURE.
    d = ManagerTestAccess::calc(*src2, steadyNowNs() + longDisconnect + kMs);
    EXPECT_EQ(d.status.state, ControlSignalState::DISCONNECTED);
    EXPECT_EQ(d.cause, LivenessCause::RESPONSE_FAILURE);
}

// S6: send after shutdown — NO_TRANSPORT; shutdown is idempotent.
TEST_F(SourceTest, S6_ShutdownIdempotent)
{
    auto src = ManagerTestAccess::createSource<Joy>(node_.get(), topicInfo("s6/joy"));
    src->shutdown();
    EXPECT_EQ(src->send(makeJoy(1.f)), SendResult::NO_TRANSPORT);
    src->shutdown();  // second call: no effect, no crash
    EXPECT_EQ(src->send(makeJoy(1.f)), SendResult::NO_TRANSPORT);
}

// S7: sendErased forwards like send; msgType() is correct.
TEST_F(SourceTest, S7_SendErased)
{
    auto src = ManagerTestAccess::createSource<Joy>(node_.get(), topicInfo("s7/joy"));
    const Joy j = makeJoy(2.f);
    EXPECT_EQ(src->sendErased(&j), SendResult::OK);
    EXPECT_EQ(src->msgType(), std::type_index(typeid(Joy)));

    const auto d = ManagerTestAccess::calc(*src, steadyNowNs());
    EXPECT_EQ(d.status.state, ControlSignalState::ACTIVE);
}

// S8: stop sending, tick past timeout — TIMEOUT; resume send, tick — ACTIVE.
TEST_F(SourceTest, S8_TimeoutThenRecover)
{
    auto src = ManagerTestAccess::createSource<Joy>(node_.get(), topicInfo("s8/joy"));
    ASSERT_EQ(src->send(makeJoy(1.f)), SendResult::OK);
    const int64_t sent = steadyNowNs();

    auto d = ManagerTestAccess::calc(*src, sent + kTimeout + kMs);
    EXPECT_EQ(d.status.state, ControlSignalState::TIMEOUT);
    ManagerTestAccess::apply(*src, d);
    EXPECT_EQ(src->getState(), ControlSignalState::TIMEOUT);

    ASSERT_EQ(src->send(makeJoy(1.f)), SendResult::OK);
    d = ManagerTestAccess::calc(*src, steadyNowNs());
    EXPECT_EQ(d.status.state, ControlSignalState::ACTIVE);
    ManagerTestAccess::apply(*src, d);
    EXPECT_EQ(src->getState(), ControlSignalState::ACTIVE);
}

// S9: tick past disconnect — _calcStatus DISCONNECTED (deregistration is the
// CSM layer's job, §8.4 M9).
TEST_F(SourceTest, S9_DisconnectDecision)
{
    auto src = ManagerTestAccess::createSource<Joy>(node_.get(), topicInfo("s9/joy"));
    ASSERT_EQ(src->send(makeJoy(1.f)), SendResult::OK);
    const int64_t sent = steadyNowNs();

    const auto d = ManagerTestAccess::calc(*src, sent + kDisconnect + kMs);
    EXPECT_EQ(d.status.state, ControlSignalState::DISCONNECTED);
    EXPECT_EQ(d.cause, LivenessCause::INACTIVITY);
}

// S10: send rate — 20 Hz for 2 s then tick: rateHz in [18, 22];
// sendRateHz() reads the same value; one idle window later it is 0.
TEST_F(SourceTest, S10_SendRate)
{
    constexpr int64_t kWindow = 1'000'000'000;  // 1 s
    auto src = ManagerTestAccess::createSource<Joy>(node_.get(), topicInfo("s10/joy"), kWindow);
    for (int i = 0; i < 40; ++i)
    {
        ASSERT_EQ(src->send(makeJoy(1.f)), SendResult::OK);
        std::this_thread::sleep_for(50ms);
    }
    const int64_t now = steadyNowNs();
    const auto d = ManagerTestAccess::calc(*src, now);
    EXPECT_GE(d.status.rateHz, 18.f);
    EXPECT_LE(d.status.rateHz, 22.f);
    ManagerTestAccess::apply(*src, d);
    EXPECT_FLOAT_EQ(src->sendRateHz(), d.status.rateHz);

    // One full idle window later (fake now): rate back to 0.
    const auto d2 = ManagerTestAccess::calc(*src, now + kWindow + 2 * kMs);
    EXPECT_FLOAT_EQ(d2.status.rateHz, 0.f);
}

// S11: rate concurrency — high-frequency send + periodic tick simulation +
// high-frequency sendRateHz(); no race (TSan job), cache converges.
TEST_F(SourceTest, S11_RateConcurrency)
{
    auto src = ManagerTestAccess::createSource<Joy>(node_.get(), topicInfo("s11/joy"));
    std::atomic<bool> stop{false};
    std::atomic<uint64_t> sent{0}, failed{0}, reads{0};
    std::atomic<bool> invalidRate{false};

    std::thread sender(
        [&]
        {
            while (!stop.load())
            {
                if (src->send(makeJoy(1.f)) == SendResult::OK)
                    sent.fetch_add(1);
                else
                    failed.fetch_add(1);
                std::this_thread::sleep_for(1ms);
            }
        });
    std::thread reader(
        [&]
        {
            while (!stop.load())
            {
                const float rate = src->sendRateHz();
                if (!std::isfinite(rate) || rate < 0.f)
                    invalidRate.store(true);
                reads.fetch_add(1);
            }
        });
    for (int i = 0; i < 50; ++i)
    {
        const auto d = ManagerTestAccess::calc(*src, steadyNowNs());
        ManagerTestAccess::apply(*src, d);
        std::this_thread::sleep_for(10ms);
    }
    stop.store(true);
    sender.join();
    reader.join();
    EXPECT_GT(sent.load(), 0u);
    EXPECT_EQ(failed.load(), 0u);
    EXPECT_GT(reads.load(), 0u);
    EXPECT_FALSE(invalidRate.load());
    EXPECT_GT(src->sendRateHz(), 0.f);

    // After the writers stop, sample at equal bucket phases so the covered
    // span is constant. The published cache must monotonically converge to
    // zero as the frozen traffic ages out of the eight-bucket window.
    const int64_t stoppedAt = steadyNowNs();
    float previous = ManagerTestAccess::calc(*src, stoppedAt).status.rateHz;
    for (int bucket = 0; bucket <= 8; ++bucket)
    {
        const auto d = ManagerTestAccess::calc(*src, stoppedAt + bucket * 125 * kMs);
        ManagerTestAccess::apply(*src, d);
        EXPECT_FLOAT_EQ(src->sendRateHz(), d.status.rateHz);
        EXPECT_LE(src->sendRateHz(), previous);
        previous = src->sendRateHz();
    }
    EXPECT_FLOAT_EQ(src->sendRateHz(), 0.f);
}

// S12: state callbacks — registered slots fire exactly once per transition
// with correct old/new on the caller (tick) thread; unregistered slots do
// nothing; nullptr clears.
TEST_F(SourceTest, S12_StateCallbacks)
{
    auto src = ManagerTestAccess::createSource<Joy>(node_.get(), topicInfo("s12/joy"));

    std::atomic<int> activeFires{0}, timeoutFires{0};
    std::thread::id cbThread;
    std::vector<ControlSignalState> activeOlds;
    src->setStateCallback(ControlSignalState::ACTIVE,
                          [&](const std::string& ctrl, ControlSignalState oldS, ControlSignalState newS)
                          {
                              EXPECT_EQ(ctrl, "ctrl_s12/joy");
                              EXPECT_EQ(newS, ControlSignalState::ACTIVE);
                              activeOlds.push_back(oldS);
                              cbThread = std::this_thread::get_id();
                              activeFires.fetch_add(1);
                          });
    src->setStateCallback(ControlSignalState::TIMEOUT,
                          [&](const std::string&, ControlSignalState oldS, ControlSignalState newS)
                          {
                              EXPECT_EQ(oldS, ControlSignalState::ACTIVE);
                              EXPECT_EQ(newS, ControlSignalState::TIMEOUT);
                              timeoutFires.fetch_add(1);
                          });

    ASSERT_EQ(src->send(makeJoy(1.f)), SendResult::OK);
    const int64_t sent = steadyNowNs();

    // INITIAL -> ACTIVE
    ManagerTestAccess::apply(*src, ManagerTestAccess::calc(*src, sent));
    EXPECT_EQ(activeFires.load(), 1);
    EXPECT_EQ(cbThread, std::this_thread::get_id());  // tick (caller) thread

    // Same state again: no fire.
    ManagerTestAccess::apply(*src, ManagerTestAccess::calc(*src, sent));
    EXPECT_EQ(activeFires.load(), 1);

    // ACTIVE -> TIMEOUT
    ManagerTestAccess::apply(*src, ManagerTestAccess::calc(*src, sent + kTimeout + kMs));
    EXPECT_EQ(timeoutFires.load(), 1);

    // TIMEOUT -> ACTIVE again
    ASSERT_EQ(src->send(makeJoy(1.f)), SendResult::OK);
    ManagerTestAccess::apply(*src, ManagerTestAccess::calc(*src, steadyNowNs()));
    EXPECT_EQ(activeFires.load(), 2);
    ASSERT_EQ(activeOlds.size(), 2u);
    EXPECT_EQ(activeOlds[0], ControlSignalState::INITIAL);
    EXPECT_EQ(activeOlds[1], ControlSignalState::TIMEOUT);

    // nullptr clears: the transition demonstrably happens, no fire.
    src->setStateCallback(ControlSignalState::TIMEOUT, nullptr);
    ManagerTestAccess::apply(*src, ManagerTestAccess::calc(*src, steadyNowNs() + kTimeout + kMs));
    EXPECT_EQ(src->getState(), ControlSignalState::TIMEOUT);  // transition occurred
    EXPECT_EQ(timeoutFires.load(), 1);  // but slot cleared
}

// S13: window configuration — 0.5 s vs 2 s windows converge on their own
// timescale; getStatus() returns consistent {state, rate}.
TEST_F(SourceTest, S13_WindowConfiguration)
{
    auto fast = ManagerTestAccess::createSource<Joy>(node_.get(), topicInfo("s13a/joy"), 500 * kMs);
    auto slow = ManagerTestAccess::createSource<Joy>(node_.get(), topicInfo("s13b/joy"), 2000 * kMs);
    // ~20 Hz for ~1 s: the 0.5 s window is fully converged (~20 Hz), the 2 s
    // window still averages over its longer span (~10 Hz).
    for (int i = 0; i < 20; ++i)
    {
        ASSERT_EQ(fast->send(makeJoy(1.f)), SendResult::OK);
        ASSERT_EQ(slow->send(makeJoy(1.f)), SendResult::OK);
        std::this_thread::sleep_for(50ms);
    }
    const int64_t now = steadyNowNs();
    const auto df = ManagerTestAccess::calc(*fast, now);
    const auto ds = ManagerTestAccess::calc(*slow, now);
    EXPECT_GE(df.status.rateHz, 16.f);
    EXPECT_LE(df.status.rateHz, 24.f);
    EXPECT_GE(ds.status.rateHz, 6.f);
    EXPECT_LE(ds.status.rateHz, 14.f);

    ManagerTestAccess::apply(*fast, df);
    const auto st = fast->getStatus();
    EXPECT_EQ(st.state, df.status.state);
    EXPECT_FLOAT_EQ(st.rateHz, df.status.rateHz);
}

// S14: out-of-order completion of concurrent service outcomes — only the
// newer request sequence updates the streak; a success clears it and bumps
// the epoch; an older timeout must not overwrite.
TEST_F(SourceTest, S14_OutOfOrderOutcomes)
{
    auto src = ManagerTestAccess::createSource<Joy, JoySrv>(node_.get(), serviceInfo("s14/joy"));
    const int64_t t0 = steadyNowNs();

    // seq 2 fails first (out-of-order: seq 1 still in flight).
    ManagerTestAccess::recordOutcomeFailure(*src, 2, t0);
    auto d = ManagerTestAccess::calc(*src, t0);
    EXPECT_EQ(d.status.state, ControlSignalState::TIMEOUT);
    const uint64_t failEpoch = d.observedFailureEpoch;

    // seq 1's late success must NOT clear the newer failure verdict.
    ManagerTestAccess::recordOutcomeSuccess(*src, 1);
    d = ManagerTestAccess::calc(*src, t0);
    EXPECT_EQ(d.status.state, ControlSignalState::TIMEOUT);
    EXPECT_EQ(d.observedFailureEpoch, failEpoch);

    // seq 3's success clears the streak and bumps the epoch.
    ManagerTestAccess::recordOutcomeSuccess(*src, 3);
    d = ManagerTestAccess::calc(*src, t0);
    EXPECT_NE(d.status.state, ControlSignalState::TIMEOUT);
    EXPECT_EQ(d.observedFailureEpoch, failEpoch + 1);

    // seq 2's duplicate/late timeout must not re-open the streak.
    ManagerTestAccess::recordOutcomeFailure(*src, 2, t0);
    d = ManagerTestAccess::calc(*src, t0);
    EXPECT_NE(d.status.state, ControlSignalState::TIMEOUT);
}

// S15: inactivity terminal seal vs send — if the seal wins, send returns
// DISCONNECTED without touching the transport; if send's activity wins
// first, the INACTIVITY commit is cancelled.
TEST_F(SourceTest, S15_InactivitySealVsSend)
{
    // (a) activity wins: seal fails, terminal cancelled.
    auto src = ManagerTestAccess::createSource<Joy>(node_.get(), topicInfo("s15a/joy"));
    ASSERT_EQ(src->send(makeJoy(1.f)), SendResult::OK);
    const int64_t sent = steadyNowNs();
    auto d = ManagerTestAccess::calc(*src, sent + kDisconnect + kMs);
    ASSERT_EQ(d.status.state, ControlSignalState::DISCONNECTED);
    ASSERT_EQ(src->send(makeJoy(1.f)), SendResult::OK);  // activity wins
    EXPECT_FALSE(ManagerTestAccess::trySealLocalTerminal(*src, d));
    EXPECT_EQ(src->getState(), ControlSignalState::INITIAL);  // nothing applied

    // (b) seal wins: send is DISCONNECTED and the transport is really left
    // untouched — a counting subscriber sees no publication.
    auto src2 = ManagerTestAccess::createSource<Joy>(node_.get(), topicInfo("s15b/joy"));
    std::atomic<int> delivered{0};
    auto counter = node_->create_subscription<Joy>("s15b/joy",
                                                   10,
                                                   [&](const Joy&)
                                                   {
                                                       delivered.fetch_add(1);
                                                   });
    ASSERT_EQ(src2->send(makeJoy(1.f)), SendResult::OK);
    const int64_t sent2 = steadyNowNs();
    for (int i = 0; i < 100 && delivered.load() < 1; ++i)
        std::this_thread::sleep_for(10ms);
    ASSERT_EQ(delivered.load(), 1);

    d = ManagerTestAccess::calc(*src2, sent2 + kDisconnect + kMs);
    ASSERT_EQ(d.status.state, ControlSignalState::DISCONNECTED);
    ASSERT_TRUE(ManagerTestAccess::trySealLocalTerminal(*src2, d));
    EXPECT_EQ(src2->send(makeJoy(1.f)), SendResult::DISCONNECTED);
    std::this_thread::sleep_for(200ms);
    EXPECT_EQ(delivered.load(), 1);  // sealed send never touched the publisher
    ManagerTestAccess::apply(*src2, d);
    EXPECT_EQ(src2->getState(), ControlSignalState::DISCONNECTED);
}

// S16: response-failure terminal — sends and failures within the same streak
// do not cancel it; a newer success response (epoch change) invalidates the
// old decision; once the seal wins first, the in-flight response path
// reports DISCONNECTED.
TEST_F(SourceTest, S16_ResponseFailureTerminal)
{
    // (a) same-streak send + failure do not cancel the terminal decision.
    // The send advances the activity generation — the RESPONSE_FAILURE seal
    // must validate the failure epoch, not the generation (§5.3: a new send
    // call is not a response recovery).
    auto src = ManagerTestAccess::createSource<Joy, JoySrv>(node_.get(), serviceInfo("s16a/joy"));
    const int64_t t0 = steadyNowNs();
    ManagerTestAccess::recordOutcomeFailure(*src, 10, t0);
    auto d = ManagerTestAccess::calc(*src, t0 + kDisconnect + kMs);
    ASSERT_EQ(d.status.state, ControlSignalState::DISCONNECTED);
    ASSERT_EQ(d.cause, LivenessCause::RESPONSE_FAILURE);
    // No server: this send fails; its seq (1) is older than the injected
    // outcome (10), so the streak record stays untouched — but the activity
    // generation moved.
    EXPECT_EQ(src->send(makeJoy(1.f)), SendResult::NO_TRANSPORT);
    ManagerTestAccess::recordOutcomeFailure(*src, 11, t0 + kMs);  // streak continues
    EXPECT_TRUE(ManagerTestAccess::trySealLocalTerminal(*src, d));

    // (b) a newer success changes the epoch first: old decision cancelled.
    auto src2 = ManagerTestAccess::createSource<Joy, JoySrv>(node_.get(), serviceInfo("s16b/joy"));
    ManagerTestAccess::recordOutcomeFailure(*src2, 1, t0);
    d = ManagerTestAccess::calc(*src2, t0 + kDisconnect + kMs);
    ASSERT_EQ(d.cause, LivenessCause::RESPONSE_FAILURE);
    ManagerTestAccess::recordOutcomeSuccess(*src2, 2);  // epoch bumps
    EXPECT_FALSE(ManagerTestAccess::trySealLocalTerminal(*src2, d));

    // (c) seal wins while a request is IN FLIGHT: the response arrives after
    // the seal and the response path — not the send preamble — must report
    // DISCONNECTED (§5.3: response 到達時 terminal seal 已勝出 → DISCONNECTED).
    const auto infoC = rv2_interfaces::r1::makeTransportInfo(
        "s16c/joy", ControlSignalInfo::MODE_SERVICE, ControlSignalInfo::TYPE_JOY, 2'000 * kMs, 20'000 * kMs);
    auto slowServer =
        node_->create_service<JoySrv>("s16c/joy",
                                      [](const std::shared_ptr<JoySrv::Request>, std::shared_ptr<JoySrv::Response> res)
                                      {
                                          std::this_thread::sleep_for(400ms);  // hold the request in flight
                                          res->response = JoySrv::Response::SRV_RES_SUCCESS;
                                      });
    auto src3 = ManagerTestAccess::createSource<Joy, JoySrv>(node_.get(), infoC);
    auto probe = node_->create_client<JoySrv>("s16c/joy");
    ASSERT_TRUE(probe->wait_for_service(3s));

    std::atomic<SendResult> inFlight{SendResult::OK};
    std::thread sender(
        [&]
        {
            inFlight.store(src3->send(makeJoy(1.f)));
        });
    std::this_thread::sleep_for(150ms);  // request is now in flight
    ManagerTestAccess::sealTerminal(*src3);  // seal wins before the response
    sender.join();
    EXPECT_EQ(inFlight.load(), SendResult::DISCONNECTED);
}

// ───────────────────────── SinkTest K1-K16 ─────────────────────────

// K1: initial — INITIAL; read() false and out untouched.
TEST_F(SinkTest, K1_Initial)
{
    auto sink = ManagerTestAccess::createSink<Joy>(node_.get(), topicInfo("k1/joy"));
    EXPECT_EQ(sink->getState(), ControlSignalState::INITIAL);
    Joy out = makeJoy(42.f);
    EXPECT_FALSE(sink->read(out));
    ASSERT_EQ(out.axes.size(), 1u);
    EXPECT_FLOAT_EQ(out.axes[0], 42.f);  // untouched default
}

// K2: first message + tick — before the tick read() is false (granularity
// semantics, §4.2); after the tick ACTIVE and read() true with content.
TEST_F(SinkTest, K2_FirstMessageGranularity)
{
    auto sink = ManagerTestAccess::createSink<Joy>(node_.get(), topicInfo("k2/joy"));
    auto pub = node_->create_publisher<Joy>("k2/joy", 10);

    std::thread waiter(
        [&]
        {
            Joy out;
            EXPECT_TRUE(sink->waitForMessage(out, 3'000 * kMs));
        });
    std::this_thread::sleep_for(100ms);
    pub->publish(makeJoy(7.f));
    waiter.join();  // message has arrived

    Joy out;
    EXPECT_FALSE(sink->read(out));  // pre-tick: still INITIAL
    ManagerTestAccess::apply(*sink, ManagerTestAccess::calc(*sink, steadyNowNs()));
    EXPECT_EQ(sink->getState(), ControlSignalState::ACTIVE);
    ASSERT_TRUE(sink->read(out));
    ASSERT_EQ(out.axes.size(), 1u);
    EXPECT_FLOAT_EQ(out.axes[0], 7.f);
}

/// Publishes one message and blocks until the sink stored it.
void publishAndWait(rclcpp::Publisher<Joy>::SharedPtr pub,
                    const std::shared_ptr<rv2_interfaces::r1::ControlSignalSink<Joy>>& sink,
                    float axis0)
{
    std::thread waiter(
        [&]
        {
            Joy out;
            EXPECT_TRUE(sink->waitForMessage(out, 3'000 * kMs));
        });
    std::this_thread::sleep_for(50ms);
    pub->publish(makeJoy(axis0));
    waiter.join();
}

// K3: stream stops, elapsed > timeout + tick — TIMEOUT; read false while
// the content stays the last value.
TEST_F(SinkTest, K3_TimeoutKeepsLastValue)
{
    auto sink = ManagerTestAccess::createSink<Joy>(node_.get(), topicInfo("k3/joy"));
    auto pub = node_->create_publisher<Joy>("k3/joy", 10);
    publishAndWait(pub, sink, 3.f);
    const int64_t got = steadyNowNs();

    ManagerTestAccess::apply(*sink, ManagerTestAccess::calc(*sink, got + kTimeout + kMs));
    EXPECT_EQ(sink->getState(), ControlSignalState::TIMEOUT);
    Joy out;
    EXPECT_FALSE(sink->read(out));

    // Content stays the last value: back to ACTIVE (activity timestamp is
    // still fresh at real now) the stored message is unchanged.
    ManagerTestAccess::apply(*sink, ManagerTestAccess::calc(*sink, steadyNowNs()));
    ASSERT_EQ(sink->getState(), ControlSignalState::ACTIVE);
    ASSERT_TRUE(sink->read(out));
    ASSERT_EQ(out.axes.size(), 1u);
    EXPECT_FLOAT_EQ(out.axes[0], 3.f);
}

// K4: receive again after TIMEOUT + tick — ACTIVE (recoverable at any time).
TEST_F(SinkTest, K4_TimeoutRecovers)
{
    auto sink = ManagerTestAccess::createSink<Joy>(node_.get(), topicInfo("k4/joy"));
    auto pub = node_->create_publisher<Joy>("k4/joy", 10);
    publishAndWait(pub, sink, 1.f);
    ManagerTestAccess::apply(*sink, ManagerTestAccess::calc(*sink, steadyNowNs() + kTimeout + kMs));
    ASSERT_EQ(sink->getState(), ControlSignalState::TIMEOUT);

    publishAndWait(pub, sink, 2.f);
    ManagerTestAccess::apply(*sink, ManagerTestAccess::calc(*sink, steadyNowNs()));
    EXPECT_EQ(sink->getState(), ControlSignalState::ACTIVE);
}

// K5: message callback — fires per message after registration, nullptr
// clears, replace semantics.
TEST_F(SinkTest, K5_MsgCallback)
{
    auto sink = ManagerTestAccess::createSink<Joy>(node_.get(), topicInfo("k5/joy"));
    auto pub = node_->create_publisher<Joy>("k5/joy", 10);

    std::atomic<int> first{0}, second{0};
    std::vector<float> seen;
    std::mutex seenMtx;
    sink->setMsgCallback(
        [&](const Joy& m, const ControlSignalInfo& info)
        {
            ASSERT_EQ(m.axes.size(), 1u);
            EXPECT_EQ(info.channel_name, "k5/joy");
            {
                std::lock_guard<std::mutex> lk(seenMtx);
                seen.push_back(m.axes[0]);
            }
            first.fetch_add(1);
        });
    publishAndWait(pub, sink, 1.f);
    publishAndWait(pub, sink, 2.f);
    EXPECT_EQ(first.load(), 2);
    {
        std::lock_guard<std::mutex> lk(seenMtx);
        ASSERT_EQ(seen.size(), 2u);
        EXPECT_FLOAT_EQ(seen[0], 1.f);
        EXPECT_FLOAT_EQ(seen[1], 2.f);
    }

    // Replace: only the new callback fires from now on.
    sink->setMsgCallback(
        [&](const Joy&, const ControlSignalInfo&)
        {
            second.fetch_add(1);
        });
    publishAndWait(pub, sink, 3.f);
    EXPECT_EQ(first.load(), 2);
    EXPECT_EQ(second.load(), 1);

    // nullptr clears.
    sink->setMsgCallback(nullptr);
    publishAndWait(pub, sink, 4.f);
    EXPECT_EQ(second.load(), 1);
}

// K6: re-entering read()/getState() inside the callback — no deadlock
// (regression for the lock-free callback invocation).
TEST_F(SinkTest, K6_CallbackReenter)
{
    auto sink = ManagerTestAccess::createSink<Joy>(node_.get(), topicInfo("k6/joy"));
    auto pub = node_->create_publisher<Joy>("k6/joy", 10);

    std::atomic<int> fires{0};
    sink->setMsgCallback(
        [&](const Joy&, const ControlSignalInfo&)
        {
            Joy out;
            (void)sink->read(out);  // re-enter under no held lock
            (void)sink->getState();
            (void)sink->getStatus();
            fires.fetch_add(1);
        });
    publishAndWait(pub, sink, 1.f);
    EXPECT_EQ(fires.load(), 1);
}

// K7: service mode round-trip — request data == read content; response
// SUCCESS.
TEST_F(SinkTest, K7_ServiceRoundTrip)
{
    auto sink = ManagerTestAccess::createSink<Joy, JoySrv>(node_.get(), serviceInfo("k7/joy"));
    auto client = node_->create_client<JoySrv>("k7/joy");
    ASSERT_TRUE(client->wait_for_service(3s));

    auto req = std::make_shared<JoySrv::Request>();
    req->data = makeJoy(9.f);
    auto fut = client->async_send_request(req);
    ASSERT_EQ(fut.wait_for(3s), std::future_status::ready);
    EXPECT_EQ(fut.get()->response, JoySrv::Response::SRV_RES_SUCCESS);

    ManagerTestAccess::apply(*sink, ManagerTestAccess::calc(*sink, steadyNowNs()));
    Joy out;
    ASSERT_TRUE(sink->read(out));
    ASSERT_EQ(out.axes.size(), 1u);
    EXPECT_FLOAT_EQ(out.axes[0], 9.f);
}

// K8: confirmed death — past disconnect the decision is DISCONNECTED; only a
// successful seal applies + deregisters, later messages are rejected; a new
// message before the seal cancels this round's terminal commit.
TEST_F(SinkTest, K8_ConfirmedDeath)
{
    // (a) new message between calc and seal: commit cancelled.
    auto sink = ManagerTestAccess::createSink<Joy>(node_.get(), topicInfo("k8a/joy"));
    auto pub = node_->create_publisher<Joy>("k8a/joy", 10);
    publishAndWait(pub, sink, 1.f);
    auto d = ManagerTestAccess::calc(*sink, steadyNowNs() + kDisconnect + kMs);
    ASSERT_EQ(d.status.state, ControlSignalState::DISCONNECTED);
    publishAndWait(pub, sink, 2.f);  // activity wins
    EXPECT_FALSE(ManagerTestAccess::trySealLocalTerminal(*sink, d));
    EXPECT_EQ(sink->getState(), ControlSignalState::INITIAL);  // nothing applied

    // (b) seal succeeds: apply, then later messages are rejected.
    auto sink2 = ManagerTestAccess::createSink<Joy>(node_.get(), topicInfo("k8b/joy"));
    auto pub2 = node_->create_publisher<Joy>("k8b/joy", 10);
    publishAndWait(pub2, sink2, 1.f);
    d = ManagerTestAccess::calc(*sink2, steadyNowNs() + kDisconnect + kMs);
    ASSERT_EQ(d.status.state, ControlSignalState::DISCONNECTED);
    ASSERT_TRUE(ManagerTestAccess::trySealLocalTerminal(*sink2, d));
    ManagerTestAccess::apply(*sink2, d);
    EXPECT_EQ(sink2->getState(), ControlSignalState::DISCONNECTED);

    std::atomic<int> lateFires{0};
    sink2->setMsgCallback(
        [&](const Joy&, const ControlSignalInfo&)
        {
            lateFires.fetch_add(1);
        });
    pub2->publish(makeJoy(3.f));
    std::this_thread::sleep_for(300ms);
    EXPECT_EQ(lateFires.load(), 0);  // sealed: no store, no callback
}

// K9: upstream keeps publishing after shutdown — no callback, no recorded
// change (the subscription is released).
TEST_F(SinkTest, K9_ShutdownStopsIntake)
{
    auto sink = ManagerTestAccess::createSink<Joy>(node_.get(), topicInfo("k9/joy"));
    auto pub = node_->create_publisher<Joy>("k9/joy", 10);
    publishAndWait(pub, sink, 1.f);

    std::atomic<int> fires{0};
    sink->setMsgCallback(
        [&](const Joy&, const ControlSignalInfo&)
        {
            fires.fetch_add(1);
        });
    sink->shutdown();
    for (int i = 0; i < 5; ++i)
    {
        pub->publish(makeJoy(2.f));
        std::this_thread::sleep_for(50ms);
    }
    EXPECT_EQ(fires.load(), 0);
}

// K10: weak-capture UAF regression — destroy the sink under high-frequency
// intake; no crash (ASan/TSan job).
TEST_F(SinkTest, K10_WeakCaptureUafRegression)
{
    auto pub = node_->create_publisher<Joy>("k10/joy", 10);
    std::atomic<bool> stop{false};
    std::thread flooder(
        [&]
        {
            while (!stop.load())
            {
                pub->publish(makeJoy(1.f));
                std::this_thread::sleep_for(1ms);
            }
        });
    int sawIntake = 0;
    for (int i = 0; i < 20; ++i)
    {
        auto sink = ManagerTestAccess::createSink<Joy>(node_.get(), topicInfo("k10/joy"));
        Joy out;
        if (sink->waitForMessage(out, 500 * kMs))
            ++sawIntake;  // prove real intake happened before destruction
        sink.reset();  // destroyed while the flood continues
    }
    stop.store(true);
    flooder.join();
    EXPECT_GT(sawIntake, 0);  // the scenario really exercised live intake
}

// K11: data rate — 20 Hz for 2 s then tick: within [18, 22]; dataRateHz()
// matches; one idle window later 0.
TEST_F(SinkTest, K11_DataRate)
{
    constexpr int64_t kWindow = 1'000'000'000;
    auto sink = ManagerTestAccess::createSink<Joy>(node_.get(), topicInfo("k11/joy"), kWindow);
    auto pub = node_->create_publisher<Joy>("k11/joy", 10);

    std::atomic<int> got{0};
    sink->setMsgCallback(
        [&](const Joy&, const ControlSignalInfo&)
        {
            got.fetch_add(1);
        });
    for (int i = 0; i < 40; ++i)
    {
        pub->publish(makeJoy(1.f));
        std::this_thread::sleep_for(50ms);
    }
    for (int i = 0; i < 100 && got.load() < 40; ++i)
        std::this_thread::sleep_for(10ms);
    ASSERT_GE(got.load(), 38);  // allow minimal transport slack

    const int64_t now = steadyNowNs();
    const auto d = ManagerTestAccess::calc(*sink, now);
    EXPECT_GE(d.status.rateHz, 18.f);
    EXPECT_LE(d.status.rateHz, 22.f);
    ManagerTestAccess::apply(*sink, d);
    EXPECT_FLOAT_EQ(sink->dataRateHz(), d.status.rateHz);

    const auto d2 = ManagerTestAccess::calc(*sink, now + kWindow + 2 * kMs);
    EXPECT_FLOAT_EQ(d2.status.rateHz, 0.f);
}

// K12: rate concurrency — high-frequency intake + periodic tick simulation +
// high-frequency dataRateHz(); no race (TSan job).
TEST_F(SinkTest, K12_RateConcurrency)
{
    auto sink = ManagerTestAccess::createSink<Joy>(node_.get(), topicInfo("k12/joy"));
    auto pub = node_->create_publisher<Joy>("k12/joy", 10);
    std::atomic<bool> stop{false};
    auto received = std::make_shared<std::atomic<uint64_t>>(0);
    sink->setMsgCallback(
        [received](const Joy&, const ControlSignalInfo&)
        {
            received->fetch_add(1);
        });
    std::atomic<uint64_t> published{0}, reads{0};
    std::atomic<bool> invalidRate{false};

    std::thread flooder(
        [&]
        {
            while (!stop.load())
            {
                pub->publish(makeJoy(1.f));
                published.fetch_add(1);
                std::this_thread::sleep_for(1ms);
            }
        });
    std::thread reader(
        [&]
        {
            while (!stop.load())
            {
                const float rate = sink->dataRateHz();
                if (!std::isfinite(rate) || rate < 0.f)
                    invalidRate.store(true);
                reads.fetch_add(1);
            }
        });
    const auto intakeDeadline = std::chrono::steady_clock::now() + 3s;
    while (received->load() == 0 && std::chrono::steady_clock::now() < intakeDeadline)
        std::this_thread::sleep_for(1ms);
    for (int i = 0; i < 50; ++i)
    {
        ManagerTestAccess::apply(*sink, ManagerTestAccess::calc(*sink, steadyNowNs()));
        std::this_thread::sleep_for(10ms);
    }
    stop.store(true);
    flooder.join();
    reader.join();
    EXPECT_GT(published.load(), 0u);
    EXPECT_GT(received->load(), 0u);
    EXPECT_GT(reads.load(), 0u);
    EXPECT_FALSE(invalidRate.load());
    EXPECT_GT(sink->dataRateHz(), 0.f);
}

// K13: waitForMessage — a message sent during the wait returns immediately
// with correct content (state granularity does not matter); the timed
// variant returns false after ~timeoutNs when nothing arrives.
TEST_F(SinkTest, K13_WaitForMessage)
{
    auto sink = ManagerTestAccess::createSink<Joy>(node_.get(), topicInfo("k13/joy"));
    auto pub = node_->create_publisher<Joy>("k13/joy", 10);

    Joy out;
    std::thread waiter(
        [&]
        {
            EXPECT_TRUE(sink->waitForMessage(out, 3'000 * kMs));
        });
    std::this_thread::sleep_for(100ms);
    const auto tPub = std::chrono::steady_clock::now();
    pub->publish(makeJoy(5.f));
    waiter.join();
    // "即刻返回": the wake must come from the notify, not from the 3 s
    // timeout expiry re-evaluating the predicate.
    EXPECT_LE(std::chrono::steady_clock::now() - tPub, 1s);
    ASSERT_EQ(out.axes.size(), 1u);
    EXPECT_FLOAT_EQ(out.axes[0], 5.f);

    // Timed variant, nothing arrives: ~timeoutNs then false.
    const auto t0 = std::chrono::steady_clock::now();
    Joy none;
    EXPECT_FALSE(sink->waitForMessage(none, 300 * kMs));
    const auto elapsed = std::chrono::steady_clock::now() - t0;
    EXPECT_GE(elapsed, 280ms);
    EXPECT_LE(elapsed, 900ms);
}

// K14: wake-up semantics — a message that existed before the call does not
// satisfy the wait (only messages arriving after); concurrent waiters all
// wake on one new message.
TEST_F(SinkTest, K14_OnlyNewMessagesWake)
{
    auto sink = ManagerTestAccess::createSink<Joy>(node_.get(), topicInfo("k14/joy"));
    auto pub = node_->create_publisher<Joy>("k14/joy", 10);
    publishAndWait(pub, sink, 1.f);  // old message stored

    Joy out;
    EXPECT_FALSE(sink->waitForMessage(out, 300 * kMs));  // old one must not trigger

    std::atomic<int> woke{0};
    std::vector<std::thread> waiters;
    for (int i = 0; i < 3; ++i)
        waiters.emplace_back(
            [&]
            {
                Joy o;
                if (sink->waitForMessage(o, 3'000 * kMs))
                    woke.fetch_add(1);
            });
    std::this_thread::sleep_for(150ms);
    const auto tPub = std::chrono::steady_clock::now();
    pub->publish(makeJoy(2.f));
    for (auto& w : waiters)
        w.join();
    // All woken by the notify itself, far below the 3 s timeout expiry.
    EXPECT_LE(std::chrono::steady_clock::now() - tPub, 1s);
    EXPECT_EQ(woke.load(), 3);  // all waiters woken by the single new message
}

// K15: waitForMessage + shutdown — waiters return false immediately; no
// deadlock, no UAF (ASan job). The sink is DESTROYED while the waiter still
// blocks, exercising the destructor's waiters_ drain (§6.3, v1.2.1).
TEST_F(SinkTest, K15_WaitInterruptedByShutdown)
{
    auto sink = ManagerTestAccess::createSink<Joy>(node_.get(), topicInfo("k15/joy"));
    auto* raw = sink.get();  // the waiter must not keep the sink alive

    std::atomic<bool> returned{false};
    std::thread waiter(
        [&]
        {
            Joy out;
            EXPECT_FALSE(raw->waitForMessage(out, 0));  // infinite wait
            returned.store(true);
        });
    const auto armedDeadline = std::chrono::steady_clock::now() + 3s;
    while (ManagerTestAccess::waiterCount(*sink) == 0 && std::chrono::steady_clock::now() < armedDeadline)
        std::this_thread::sleep_for(1ms);
    const bool armed = ManagerTestAccess::waiterCount(*sink) == 1;
    if (!armed)
    {
        // Keep ownership until a delayed waiter has entered and returned.
        // Never destroy the raw pointer's pointee on a failed precondition.
        sink->shutdown();
        waiter.join();
        FAIL() << "waitForMessage never entered its waiter fence";
    }
    EXPECT_FALSE(returned.load());
    const auto t0 = std::chrono::steady_clock::now();
    sink.reset();  // ~ControlSignalSink: shutdown-wake + drain waiters_
    waiter.join();
    EXPECT_LE(std::chrono::steady_clock::now() - t0, 1s);  // immediate return
    EXPECT_TRUE(returned.load());
}

// K16: terminal seal vs receive callback interleave — activity first: no
// deregistration; seal first: no store, no wakeup, no callback (TSan job).
TEST_F(SinkTest, K16_SealVsReceiveInterleave)
{
    // (a) activity wins (same shape as K8a, asserted via callback).
    auto sink = ManagerTestAccess::createSink<Joy>(node_.get(), topicInfo("k16a/joy"));
    auto pub = node_->create_publisher<Joy>("k16a/joy", 10);
    publishAndWait(pub, sink, 1.f);
    auto d = ManagerTestAccess::calc(*sink, steadyNowNs() + kDisconnect + kMs);
    ASSERT_EQ(d.status.state, ControlSignalState::DISCONNECTED);
    publishAndWait(pub, sink, 2.f);
    EXPECT_FALSE(ManagerTestAccess::trySealLocalTerminal(*sink, d));

    // (b) seal wins: no store, no wakeup, no callback.
    auto sink2 = ManagerTestAccess::createSink<Joy>(node_.get(), topicInfo("k16b/joy"));
    auto pub2 = node_->create_publisher<Joy>("k16b/joy", 10);
    publishAndWait(pub2, sink2, 1.f);
    d = ManagerTestAccess::calc(*sink2, steadyNowNs() + kDisconnect + kMs);
    ASSERT_TRUE(ManagerTestAccess::trySealLocalTerminal(*sink2, d));

    std::atomic<int> fires{0};
    sink2->setMsgCallback(
        [&](const Joy&, const ControlSignalInfo&)
        {
            fires.fetch_add(1);
        });
    std::thread waiter(
        [&]
        {
            Joy o;
            EXPECT_FALSE(sink2->waitForMessage(o, 400 * kMs));  // never woken
        });
    std::this_thread::sleep_for(50ms);
    pub2->publish(makeJoy(9.f));
    waiter.join();
    EXPECT_EQ(fires.load(), 0);

    // (c) Real competing threads execute the production receive hot path and
    // terminal CAS. Keep (a)/(b)'s DDS proofs; this direct friend invocation
    // removes the scheduler gap between publishing and actually receiving.
    for (int round = 0; round < 64; ++round)
    {
        auto racingSink =
            ManagerTestAccess::createSink<Joy>(node_.get(), topicInfo("k16race/joy_" + std::to_string(round)));
        ManagerTestAccess::store(*racingSink, makeJoy(1.f));
        ManagerTestAccess::apply(*racingSink, ManagerTestAccess::calc(*racingSink, steadyNowNs()));
        const auto terminal = ManagerTestAccess::calc(*racingSink, steadyNowNs() + kDisconnect + kMs);
        ASSERT_EQ(terminal.status.state, ControlSignalState::DISCONNECTED);

        std::atomic<int> ready{0};
        std::atomic<bool> go{false};
        std::atomic<int> accepted{0};
        racingSink->setMsgCallback(
            [&](const Joy&, const ControlSignalInfo&)
            {
                accepted.fetch_add(1);
            });
        bool sealed = false;
        std::thread receiver(
            [&]
            {
                ready.fetch_add(1);
                while (!go.load())
                    std::this_thread::yield();
                ManagerTestAccess::store(*racingSink, makeJoy(2.f));
            });
        std::thread sealer(
            [&]
            {
                ready.fetch_add(1);
                while (!go.load())
                    std::this_thread::yield();
                sealed = ManagerTestAccess::trySealLocalTerminal(*racingSink, terminal);
            });
        const auto readyDeadline = std::chrono::steady_clock::now() + 3s;
        while (ready.load() != 2 && std::chrono::steady_clock::now() < readyDeadline)
            std::this_thread::yield();
        const bool bothReady = ready.load() == 2;
        go.store(true);
        receiver.join();
        sealer.join();
        ASSERT_TRUE(bothReady);
        EXPECT_EQ(accepted.load(), sealed ? 0 : 1);

        // Seal changes only the activity fence, not the cached ACTIVE state,
        // so read() exposes whether the racing receive stored its payload.
        Joy latest;
        ASSERT_TRUE(racingSink->read(latest));
        ASSERT_EQ(latest.axes.size(), 1u);
        EXPECT_FLOAT_EQ(latest.axes[0], sealed ? 1.f : 2.f);
        ManagerTestAccess::sealTerminal(*racingSink);
        const int beforeRejected = accepted.load();
        ManagerTestAccess::store(*racingSink, makeJoy(9.f));
        EXPECT_EQ(accepted.load(), beforeRejected);
        ASSERT_TRUE(racingSink->read(latest));
        EXPECT_FLOAT_EQ(latest.axes[0], sealed ? 1.f : 2.f);
        racingSink->setMsgCallback(nullptr);
    }
}

}  // namespace

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    ::testing::AddGlobalTestEnvironment(new rv2_interfaces::r1::RclcppEnv);
    return RUN_ALL_TESTS();
}
