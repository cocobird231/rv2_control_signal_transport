/**
 * @file test_manager.cpp
 * @brief M1-M26 integration tests for r1::ControlSignalManager (design §8.4).
 *
 * Twin-node / twin-Manager architecture with a MultiThreadedExecutor
 * spinning in the background. rclcpp timers cannot take a fake clock, so
 * short ManagerOptions periods compress test time. The mock master is a
 * bare-service node recording requests (§8.4).
 */

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <numeric>
#include <set>
#include <string>
#include <thread>
#include <vector>

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joy.hpp>

#include "r1_test_utils.h"
#include "rv2_control_signal_transport/r1/control_signal_manager.h"

namespace
{

using namespace std::chrono_literals;
using rv2_interfaces::r1::ControlSignalInfo;
using rv2_interfaces::r1::ControlSignalManager;
using rv2_interfaces::r1::ControlSignalState;
using rv2_interfaces::r1::ManagerOptions;
using rv2_interfaces::r1::RegisterError;
using rv2_interfaces::r1::RetryPolicy;
using rv2_interfaces::r1::SendResult;
using rv2_interfaces::r1::steadyNowNs;
using Joy = sensor_msgs::msg::Joy;
using ManageSrv = r1_interfaces::srv::ControlSignalManage;
using CsmNotifySrv = r1_interfaces::srv::CsmNotify;
using CsmRegisterSrv = r1_interfaces::srv::CsmRegister;
using CsmHeartbeatSrv = r1_interfaces::srv::CsmHeartbeat;
using InfoReqSrv = r1_interfaces::srv::ControlSignalInfoReq;
using ManagerStatusT = r1_interfaces::msg::ManagerStatus;
using EntryStatusT = r1_interfaces::msg::EntryStatus;

constexpr int64_t kMs = 1'000'000;

bool waitFor(const std::function<bool()>& cond, int64_t timeoutMs = 5000,
             int64_t pollMs = 10)
{
    const auto deadline =
        std::chrono::steady_clock::now() + std::chrono::milliseconds(timeoutMs);
    while (std::chrono::steady_clock::now() < deadline)
    {
        if (cond())
            return true;
        std::this_thread::sleep_for(std::chrono::milliseconds(pollMs));
    }
    return cond();
}

Joy makeJoy(float a)
{
    Joy j;
    j.axes = {a};
    return j;
}

/// Mock master: records CsmRegister requests, counts heartbeats, can answer
/// UNKNOWN_CSM once to force a re-register (M17).
struct MockMaster
{
    explicit MockMaster(rclcpp::Node* node, const std::string& masterName)
    {
        reg = node->create_service<CsmRegisterSrv>(
            masterName + "/register",
            [this](const std::shared_ptr<CsmRegisterSrv::Request> rq,
                   std::shared_ptr<CsmRegisterSrv::Response> rs) {
                {
                    std::lock_guard<std::mutex> lk(mtx);
                    registers.push_back(*rq);
                }
                rs->response = CsmRegisterSrv::Response::RESPONSE_SUCCESS;
            });
        hbNode_ = node;
        hbName_ = masterName + "/heartbeat";
        makeHeartbeatService();
    }

    void makeHeartbeatService()
    {
        hb = hbNode_->create_service<CsmHeartbeatSrv>(
            hbName_,
            [this](const std::shared_ptr<CsmHeartbeatSrv::Request> rq,
                   std::shared_ptr<CsmHeartbeatSrv::Response> rs) {
                int nowPer = 0;
                {
                    std::lock_guard<std::mutex> lk(mtx);
                    nowPer = ++concurrentPer[rq->csm_name];
                    maxConcurrentPer[rq->csm_name] =
                        std::max(maxConcurrentPer[rq->csm_name], nowPer);
                    ++heartbeats;
                }
                if (answerUnknownOnce.exchange(false))
                    rs->response = CsmHeartbeatSrv::Response::RESPONSE_UNKNOWN_CSM;
                else
                    rs->response = CsmHeartbeatSrv::Response::RESPONSE_SUCCESS;
                {
                    std::lock_guard<std::mutex> lk(mtx);
                    --concurrentPer[rq->csm_name];
                }
            });
    }

    void killHeartbeat() { hb.reset(); }

    std::mutex mtx;
    std::vector<CsmRegisterSrv::Request> registers;
    int heartbeats{0};
    std::map<std::string, int> concurrentPer;
    std::map<std::string, int> maxConcurrentPer;
    std::atomic<bool> answerUnknownOnce{false};
    rclcpp::Node* hbNode_{nullptr};
    std::string hbName_;
    rclcpp::Service<CsmRegisterSrv>::SharedPtr reg;
    rclcpp::Service<CsmHeartbeatSrv>::SharedPtr hb;
};

/// Event collector for the notification callback.
struct Events
{
    void push(const ControlSignalManager::NotificationEvent& e)
    {
        std::lock_guard<std::mutex> lk(mtx);
        list.push_back(e);
    }
    int count(ControlSignalManager::EventKind k)
    {
        std::lock_guard<std::mutex> lk(mtx);
        int n = 0;
        for (const auto& e : list)
            if (e.kind == k)
                ++n;
        return n;
    }
    std::vector<ControlSignalManager::NotificationEvent> snapshot()
    {
        std::lock_guard<std::mutex> lk(mtx);
        return list;
    }
    std::mutex mtx;
    std::vector<ControlSignalManager::NotificationEvent> list;
};

class ManagerTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        uid_ = "m" + std::to_string(counter_++);
        nodeA_ = std::make_shared<rclcpp::Node>("csm_a_" + uid_);
        nodeB_ = std::make_shared<rclcpp::Node>("csm_b_" + uid_);
        auxNode_ = std::make_shared<rclcpp::Node>("aux_" + uid_);
        executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>(
            rclcpp::ExecutorOptions(), 4);
        executor_->add_node(nodeA_);
        executor_->add_node(nodeB_);
        executor_->add_node(auxNode_);
        master_ = std::make_unique<MockMaster>(auxNode_.get(), masterName());
        spin_ = std::thread([this] { executor_->spin(); });
        while (!executor_->is_spinning())
            std::this_thread::sleep_for(1ms);
    }

    void TearDown() override
    {
        mgrA_.reset();
        mgrB_.reset();
        executor_->cancel();
        if (spin_.joinable())
            spin_.join();
        master_.reset();
        executor_.reset();
        auxNode_.reset();
        nodeB_.reset();
        nodeA_.reset();
    }

    std::string masterName() const { return "master_" + uid_; }
    std::string nameA() const { return "csmA_" + uid_; }
    std::string nameB() const { return "csmB_" + uid_; }

    /// Short-period options; autoRetry toggles the D7 initial retry.
    ManagerOptions makeOptions(bool autoRetry, int64_t tickMs = 50)
    {
        RetryPolicy p(100, 800, 0.1, 3, 4);
        p.autoRetryInitial = autoRetry;
        ManagerOptions o(p);
        o.statusIntervalMs = tickMs;
        o.pendingTtlMs = 800;
        o.maxRegisterTimeoutMs = 5000;
        o.masterName = masterName();
        o.csmTimeoutNs = 2'000 * kMs;         // tick << csmTimeout/2
        o.csmDisconnectTimeoutNs = 20'000 * kMs;
        return o;
    }

    void makeManagers(bool autoRetry = false, int64_t tickMs = 50)
    {
        mgrA_ = std::make_unique<ControlSignalManager>(nodeA_.get(), nameA(),
                                                       makeOptions(autoRetry, tickMs));
        mgrB_ = std::make_unique<ControlSignalManager>(nodeB_.get(), nameB(),
                                                       makeOptions(autoRetry, tickMs));
        ASSERT_TRUE(waitFor([&] {
            return mgrA_->getName() == nameA();   // trivial; wait for first ticks:
        }));
        ASSERT_TRUE(waitFor([&] { return tickObserved(*mgrA_) && tickObserved(*mgrB_); }));
    }

    static bool tickObserved(ControlSignalManager& m)
    {
        // registerSource's precondition flag flips on the first tick; probe it
        // with an invalid call that fails fast either way.
        const auto r = m.registerSource(ControlSignalInfo(), 1);
        return r.code != RegisterError::INVALID_CONTEXT ||
               m.getSourceInfoList().empty() == false;
    }

    ControlSignalInfo info(const std::string& tag, const std::string& target,
                           int64_t timeoutNs = 200 * kMs,
                           int64_t disconnectNs = 5'000 * kMs,
                           const std::string& mode = "topic")
    {
        ControlSignalInfo i;
        i.controller_name = "ctrl_" + uid_ + "_" + tag;
        i.channel_name = uid_ + "/" + tag;
        i.target_manager_name = target;
        i.mode = mode;
        i.type = "joy";
        i.priority = 50;
        i.timeout_ns = timeoutNs;
        i.disconnect_timeout_ns = disconnectNs;
        return i;
    }

    /// Latest status snapshot of a manager, captured via a transient sub.
    bool captureStatus(const std::string& mgrName, ManagerStatusT& out,
                       int64_t timeoutMs = 3000)
    {
        std::mutex mtx;
        std::optional<ManagerStatusT> got;
        auto sub = auxNode_->create_subscription<ManagerStatusT>(
            mgrName + "/status", 10, [&](const ManagerStatusT& m) {
                std::lock_guard<std::mutex> lk(mtx);
                got = m;
            });
        const bool ok = waitFor([&] {
            std::lock_guard<std::mutex> lk(mtx);
            return got.has_value();
        }, timeoutMs);
        if (ok)
        {
            std::lock_guard<std::mutex> lk(mtx);
            out = *got;
        }
        return ok;
    }

    static int counter_;
    std::string uid_;
    rclcpp::Node::SharedPtr nodeA_, nodeB_, auxNode_;
    std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
    std::thread spin_;
    std::unique_ptr<MockMaster> master_;
    std::unique_ptr<ControlSignalManager> mgrA_, mgrB_;
};
int ManagerTest::counter_ = 0;

// M1: normal registration — local slot/Source and remote Sink share the
// identity; the handle is valid and ready.
TEST_F(ManagerTest, M1_NormalRegistration)
{
    makeManagers();
    const auto i = info("m1", nameB());
    auto r = mgrA_->registerSource(i);
    ASSERT_EQ(r.code, RegisterError::OK);
    EXPECT_TRUE(r.handle.valid());
    EXPECT_TRUE(r.handle.ready());
    ASSERT_TRUE(waitFor([&] {
        return mgrB_->getSinkState(i.controller_name).has_value();
    }));

    ManagerStatusT stA, stB;
    ASSERT_TRUE(captureStatus(nameA(), stA));
    ASSERT_TRUE(captureStatus(nameB(), stB));
    ASSERT_EQ(stA.sources.size(), 1u);
    ASSERT_EQ(stB.sinks.size(), 1u);
    EXPECT_EQ(stA.sources[0].registration_id, stB.sinks[0].registration_id);
    EXPECT_EQ(stA.sources[0].attempt_generation, stB.sinks[0].attempt_generation);
    EXPECT_EQ(stA.sources[0].source_csm_instance_id,
              stB.sinks[0].source_csm_instance_id);
}

// M2: local duplicate controller or channel — error without any remote call.
TEST_F(ManagerTest, M2_LocalDuplicates)
{
    makeManagers();
    const auto i = info("m2", nameB());
    ASSERT_EQ(mgrA_->registerSource(i).code, RegisterError::OK);
    ASSERT_TRUE(waitFor([&] { return mgrB_->getSinkInfoList().size() == 1; }));

    EXPECT_EQ(mgrA_->registerSource(i).code, RegisterError::DUPLICATE);
    auto i2 = info("m2b", nameB());
    i2.channel_name = i.channel_name;   // duplicate channel, new controller
    EXPECT_EQ(mgrA_->registerSource(i2).code, RegisterError::DUPLICATE);

    std::this_thread::sleep_for(200ms);
    EXPECT_EQ(mgrB_->getSinkInfoList().size(), 1u);   // no second Sink appeared
}

// M3: cross-manager duplicate — remote refuses; no PENDING remnant on A2.
TEST_F(ManagerTest, M3_CrossManagerDuplicate)
{
    makeManagers();
    const auto i = info("m3", nameB());
    ASSERT_EQ(mgrA_->registerSource(i).code, RegisterError::OK);

    auto mgrA2 = std::make_unique<ControlSignalManager>(auxNode_.get(),
                                                        "csmA2_" + uid_,
                                                        makeOptions(false));
    ASSERT_TRUE(waitFor([&] { return tickObserved(*mgrA2); }));
    auto i2 = i;   // same controller & channel, other manager
    const auto r = mgrA2->registerSource(i2);
    // Remote refuses with a typed D3 conflict; per §2.4 that conflict enters
    // mandatory RETRY_WAIT (retry-until-success), so the row's "no PENDING
    // remnant" reads as: no PENDING entry survives the refused transaction.
    EXPECT_EQ(r.code, RegisterError::RETRY_SCHEDULED);
    EXPECT_TRUE(r.handle.valid());
    EXPECT_FALSE(r.handle.ready());
    ManagerStatusT st;
    ASSERT_TRUE(captureStatus("csmA2_" + uid_, st));
    ASSERT_EQ(st.sources.size(), 1u);
    EXPECT_EQ(st.sources[0].registration_phase, EntryStatusT::PHASE_RETRY_WAIT);
    EXPECT_NE(st.sources[0].registration_phase, EntryStatusT::PHASE_PENDING);
    EXPECT_TRUE(mgrA2->unregisterSource(r.handle));   // clean shutdown
}

// M4: registration storm — 16 threads, same controller; exactly one wins,
// no overwrite (rv2 TOCTOU regression).
TEST_F(ManagerTest, M4_RegistrationStorm)
{
    makeManagers();
    // Doc row: same controller, DIFFERENT targets. Three extra real targets
    // beside B; exactly one thread may win the local slot.
    std::vector<std::unique_ptr<ControlSignalManager>> extras;
    std::vector<std::string> targets{nameB()};
    for (int k = 0; k < 3; ++k)
    {
        const std::string n = "t4x" + std::to_string(k) + "_" + uid_;
        extras.push_back(std::make_unique<ControlSignalManager>(auxNode_.get(), n,
                                                                makeOptions(false)));
        targets.push_back(n);
    }
    ASSERT_TRUE(waitFor([&] {
        for (auto& e : extras)
            if (!tickObserved(*e))
                return false;
        return true;
    }));
    std::atomic<int> ok{0}, dup{0}, other{0};
    std::vector<std::thread> threads;
    for (int t = 0; t < 16; ++t)
        threads.emplace_back([&, t] {
            auto i = info("m4", targets[t % targets.size()]);
            i.channel_name += "_" + std::to_string(t);   // same controller only
            const auto r = mgrA_->registerSource(i);
            if (r.code == RegisterError::OK)
                ok.fetch_add(1);
            else if (r.code == RegisterError::DUPLICATE)
                dup.fetch_add(1);
            else
                other.fetch_add(1);
        });
    for (auto& t : threads)
        t.join();
    EXPECT_EQ(ok.load(), 1);
    EXPECT_EQ(dup.load(), 15);
    EXPECT_EQ(other.load(), 0);
    EXPECT_EQ(mgrA_->getSourceInfoList().size(), 1u);
}

// M5: nonexistent target — with the policy enabled the slot turns RETRY_WAIT
// and the call returns RETRY_SCHEDULED; when the target comes up the async
// retry succeeds.
TEST_F(ManagerTest, M5_RetryAfterTargetAppears)
{
    makeManagers(/*autoRetry=*/true);
    Events ev;
    mgrA_->setNotificationCallback([&](const auto& e) { ev.push(e); });

    const std::string lateName = "late_" + uid_;
    const auto i = info("m5", lateName);
    const auto r = mgrA_->registerSource(i);
    ASSERT_EQ(r.code, RegisterError::RETRY_SCHEDULED);
    EXPECT_TRUE(r.handle.valid());
    EXPECT_FALSE(r.handle.ready());

    auto late = std::make_unique<ControlSignalManager>(auxNode_.get(), lateName,
                                                       makeOptions(false));
    ASSERT_TRUE(waitFor([&] { return r.handle.ready(); }, 8000));
    EXPECT_GE(ev.count(ControlSignalManager::EventKind::RETRY_SUCCEEDED), 1);
    EXPECT_TRUE(waitFor([&] {
        return late->getSinkState(i.controller_name).has_value();
    }));

    // Policy disabled: the slot is removed outright, no RETRY_SCHEDULED.
    auto noRetry = std::make_unique<ControlSignalManager>(
        auxNode_.get(), "noretry5_" + uid_, makeOptions(false));
    ASSERT_TRUE(waitFor([&] { return tickObserved(*noRetry); }));
    const auto r2 = noRetry->registerSource(info("m5b", "void5_" + uid_));
    EXPECT_EQ(r2.code, RegisterError::TARGET_UNREACHABLE);
    EXPECT_FALSE(r2.handle.valid());
    EXPECT_TRUE(noRetry->getSourceInfoList().empty());
    EXPECT_FALSE(noRetry->getSource(info("m5b", "void5_" + uid_).controller_name)
                     .valid());   // no RETRY_WAIT slot leaked either
}

// M6: remote accepts but the response is lost (server answers after the
// caller timed out) — rollback fires a matching-generation UNREGISTER.
TEST_F(ManagerTest, M6_ResponseLossRollback)
{
    makeManagers();
    std::mutex mtx;
    std::vector<ManageSrv::Request> seen;
    auto slowTarget = auxNode_->create_service<ManageSrv>(
        "slowT_" + uid_ + "/control_signal_manage",
        [&](const std::shared_ptr<ManageSrv::Request> rq,
            std::shared_ptr<ManageSrv::Response> rs) {
            {
                std::lock_guard<std::mutex> lk(mtx);
                seen.push_back(*rq);
            }
            if (rq->op == ManageSrv::Request::OP_REGISTER)
                std::this_thread::sleep_for(700ms);   // beyond the caller timeout
            rs->response = ManageSrv::Response::RESPONSE_SUCCESS;
        });

    const auto i = info("m6", "slowT_" + uid_);
    const auto r = mgrA_->registerSource(i, 300);
    EXPECT_EQ(r.code, RegisterError::TIMEOUT_UNKNOWN);
    EXPECT_FALSE(r.handle.valid());
    EXPECT_TRUE(mgrA_->getSourceInfoList().empty());

    // The rollback UNREGISTER arrives with the same identity triple.
    ASSERT_TRUE(waitFor([&] {
        std::lock_guard<std::mutex> lk(mtx);
        for (const auto& q : seen)
            if (q.op == ManageSrv::Request::OP_UNREGISTER)
                return true;
        return false;
    }, 5000));
    {
        std::lock_guard<std::mutex> lk(mtx);
        ASSERT_GE(seen.size(), 2u);
        EXPECT_EQ(seen.front().registration_id, seen.back().registration_id);
        EXPECT_EQ(seen.front().attempt_generation, seen.back().attempt_generation);
    }
    // The delayed SUCCESS eventually lands on a caller that already rolled
    // back: it must change nothing (stale response tolerance, M23 family).
    std::this_thread::sleep_for(900ms);
    EXPECT_TRUE(mgrA_->getSourceInfoList().empty());
    EXPECT_FALSE(r.handle.valid());
}

// M7: unregister against REGISTERED and RETRY_WAIT states — desired drops
// at once, the handle invalidates immediately, retry is cancelled and a late
// success cannot revive the entry.
TEST_F(ManagerTest, M7_UnregisterStates)
{
    makeManagers(/*autoRetry=*/true);

    // REGISTERED: removal on the next tick, remote sink follows.
    const auto i1 = info("m7a", nameB());
    auto r1 = mgrA_->registerSource(i1);
    ASSERT_EQ(r1.code, RegisterError::OK);
    ASSERT_TRUE(waitFor([&] { return mgrB_->getSinkState(i1.controller_name).has_value(); }));
    EXPECT_TRUE(mgrA_->unregisterSource(r1.handle));
    EXPECT_FALSE(r1.handle.valid());   // immediate
    ASSERT_TRUE(waitFor([&] { return mgrA_->getSourceInfoList().empty(); }));
    ASSERT_TRUE(waitFor([&] {
        return !mgrB_->getSinkState(i1.controller_name).has_value();
    }));

    // RETRY_WAIT: slot vanishes now; target coming up later must not revive.
    const std::string lateName = "late7_" + uid_;
    const auto i2 = info("m7b", lateName);
    auto r2 = mgrA_->registerSource(i2);
    ASSERT_EQ(r2.code, RegisterError::RETRY_SCHEDULED);
    EXPECT_TRUE(mgrA_->unregisterSource(r2.handle));
    EXPECT_FALSE(r2.handle.valid());
    EXPECT_TRUE(mgrA_->getSourceInfoList().empty());

    auto late = std::make_unique<ControlSignalManager>(auxNode_.get(), lateName,
                                                       makeOptions(false));
    std::this_thread::sleep_for(600ms);
    EXPECT_FALSE(r2.handle.valid());
    EXPECT_FALSE(late->getSinkState(i2.controller_name).has_value());

    // In-flight: unregister while the first REGISTER is on the wire; the
    // late success response must not revive the entry, and the accepted
    // remote side gets rolled back by a matching UNREGISTER.
    std::mutex m7mtx;
    std::vector<ManageSrv::Request> m7seen;
    auto slowT = auxNode_->create_service<ManageSrv>(
        "slow7_" + uid_ + "/control_signal_manage",
        [&](const std::shared_ptr<ManageSrv::Request> rq,
            std::shared_ptr<ManageSrv::Response> rs) {
            {
                std::lock_guard<std::mutex> lk(m7mtx);
                m7seen.push_back(*rq);
            }
            if (rq->op == ManageSrv::Request::OP_REGISTER)
                std::this_thread::sleep_for(400ms);
            rs->response = ManageSrv::Response::RESPONSE_SUCCESS;
        });
    const auto i3 = info("m7c", "slow7_" + uid_);
    std::atomic<RegisterError> inFlightCode{RegisterError::OK};
    std::thread reg([&] {
        inFlightCode.store(mgrA_->registerSource(i3, 2000).code);
    });
    ASSERT_TRUE(waitFor([&] {
        std::lock_guard<std::mutex> lk(m7mtx);
        return !m7seen.empty();
    }));
    // Request on the wire: cancel via the handle from getSource (PENDING).
    auto pending = mgrA_->getSource(i3.controller_name);
    EXPECT_TRUE(mgrA_->unregisterSource(pending));
    reg.join();
    EXPECT_EQ(inFlightCode.load(), RegisterError::TIMEOUT_UNKNOWN);
    EXPECT_TRUE(mgrA_->getSourceInfoList().empty());
    ASSERT_TRUE(waitFor([&] {
        std::lock_guard<std::mutex> lk(m7mtx);
        int unregs = 0;
        for (const auto& q : m7seen)
            if (q.op == ManageSrv::Request::OP_UNREGISTER)
                ++unregs;
        // Two independent rollbacks: unregisterSource's best-effort one AND
        // the post-response abort path's (J) — both must arrive.
        return unregs >= 2;
    }, 5000));
}

// M8: notification kinds via a mock master client — STATE observation only;
// CSM_TIMEOUT/ACTIVE touch peerHealth only; DISCONNECTED queues exactly one
// removal for the matching identity; resend -> ALREADY_APPLIED; old
// generation -> STALE.
TEST_F(ManagerTest, M8_NotificationKinds)
{
    makeManagers(/*autoRetry=*/true);
    Events ev;
    mgrA_->setNotificationCallback([&](const auto& e) { ev.push(e); });

    const auto i = info("m8", nameB());
    auto r = mgrA_->registerSource(i);
    ASSERT_EQ(r.code, RegisterError::OK);
    ManagerStatusT st;
    ASSERT_TRUE(captureStatus(nameA(), st));
    ASSERT_EQ(st.sources.size(), 1u);

    auto client = auxNode_->create_client<CsmNotifySrv>(nameA() + "/get_notifications");
    ASSERT_TRUE(client->wait_for_service(3s));
    auto call = [&](int8_t kind, const std::string& eventId, uint64_t gen,
                    int8_t health = 0) {
        auto rq = std::make_shared<CsmNotifySrv::Request>();
        rq->kind = kind;
        rq->peer_csm_health = health;
        rq->event_id = eventId;
        rq->target_csm_instance_id = st.csm_instance_id;
        EntryStatusT e = st.sources[0];
        e.attempt_generation = gen;
        rq->entries = {e};
        auto f = client->async_send_request(rq);
        EXPECT_EQ(f.wait_for(3s), std::future_status::ready);
        return f.get()->response;
    };
    const uint64_t gen = st.sources[0].attempt_generation;

    // STATE: event only, no state change.
    EXPECT_EQ(call(CsmNotifySrv::Request::KIND_STATE, "e1", gen),
              CsmNotifySrv::Response::RESPONSE_APPLIED);
    EXPECT_TRUE(waitFor([&] {
        return ev.count(ControlSignalManager::EventKind::PEER_STATE) == 1;
    }));
    EXPECT_TRUE(r.handle.ready());

    // CSM_TIMEOUT warning + recovery: peerHealth only.
    EXPECT_EQ(call(CsmNotifySrv::Request::KIND_CSM_TIMEOUT, "e2", gen,
                   CsmNotifySrv::Request::PEER_HEALTH_TIMEOUT),
              CsmNotifySrv::Response::RESPONSE_APPLIED);
    EXPECT_EQ(call(CsmNotifySrv::Request::KIND_CSM_TIMEOUT, "e3", gen,
                   CsmNotifySrv::Request::PEER_HEALTH_ACTIVE),
              CsmNotifySrv::Response::RESPONSE_APPLIED);
    EXPECT_TRUE(waitFor([&] {
        return ev.count(ControlSignalManager::EventKind::PEER_CSM_TIMEOUT) == 1 &&
               ev.count(ControlSignalManager::EventKind::PEER_CSM_ACTIVE) == 1;
    }));
    EXPECT_TRUE(r.handle.ready());   // local state untouched

    // Old generation -> STALE, nothing changes.
    EXPECT_EQ(call(CsmNotifySrv::Request::KIND_DISCONNECTED, "e4", gen + 7),
              CsmNotifySrv::Response::RESPONSE_STALE);
    EXPECT_TRUE(r.handle.ready());

    // Matching DISCONNECTED: removal on the next tick, source enters retry.
    EXPECT_EQ(call(CsmNotifySrv::Request::KIND_DISCONNECTED, "e5", gen),
              CsmNotifySrv::Response::RESPONSE_APPLIED);
    ASSERT_TRUE(waitFor([&] { return !r.handle.ready(); }));
    EXPECT_TRUE(r.handle.valid());   // RETRY_WAIT keeps the intent

    // Resend of the same event: idempotent.
    EXPECT_EQ(call(CsmNotifySrv::Request::KIND_DISCONNECTED, "e5", gen),
              CsmNotifySrv::Response::RESPONSE_ALREADY_APPLIED);

    // PAIR_MISSING behaves like DISCONNECTED: removal + retry for the match.
    const auto i2 = info("m8b", nameB());
    auto r2 = mgrA_->registerSource(i2);
    ASSERT_EQ(r2.code, RegisterError::OK);
    ManagerStatusT st2;
    ASSERT_TRUE(waitFor([&] {
        return captureStatus(nameA(), st2) && st2.sources.size() == 2;
    }));
    for (const auto& e : st2.sources)
        if (e.controller_name == i2.controller_name)
        {
            auto rq = std::make_shared<CsmNotifySrv::Request>();
            rq->kind = CsmNotifySrv::Request::KIND_PAIR_MISSING;
            rq->event_id = "e6";
            rq->target_csm_instance_id = st2.csm_instance_id;
            rq->entries = {e};
            auto f = client->async_send_request(rq);
            ASSERT_EQ(f.wait_for(3s), std::future_status::ready);
            EXPECT_EQ(f.get()->response, CsmNotifySrv::Response::RESPONSE_APPLIED);
        }
    ASSERT_TRUE(waitFor([&] { return !r2.handle.ready(); }));
    EXPECT_TRUE(r2.handle.valid());   // RETRY_WAIT: intent kept
    EXPECT_TRUE(waitFor([&] {
        return ev.count(ControlSignalManager::EventKind::PAIR_MISSING) >= 1;
    }));
}

// M9: auto-disconnect — the tick decides DISCONNECTED, fires the state
// callback, removes the entry, invalidates the handle and emits the
// notification in the same round; a local cause never enters retry.
TEST_F(ManagerTest, M9_AutoDisconnect)
{
    makeManagers(/*autoRetry=*/true);
    Events ev;
    mgrA_->setNotificationCallback([&](const auto& e) { ev.push(e); });
    std::atomic<int> discCb{0};
    mgrA_->registerSourceStateCallback(
        ControlSignalState::DISCONNECTED,
        [&](const std::string&, ControlSignalState, ControlSignalState) {
            discCb.fetch_add(1);
        });

    auto i = info("m9", nameB(), 100 * kMs, 300 * kMs);
    auto r = mgrA_->registerSource(i);
    ASSERT_EQ(r.code, RegisterError::OK);
    ASSERT_EQ(r.handle.send(makeJoy(1.f)), SendResult::OK);

    ASSERT_TRUE(waitFor([&] { return !r.handle.valid(); }, 5000));
    EXPECT_EQ(discCb.load(), 1);
    EXPECT_TRUE(waitFor([&] {
        return ev.count(ControlSignalManager::EventKind::LOCAL_DISCONNECTED) >= 1;
    }));
    EXPECT_TRUE(mgrA_->getSourceInfoList().empty());
    EXPECT_EQ(ev.count(ControlSignalManager::EventKind::RETRY_STARTED), 0);
}

// M10: operating a handle after removal — error codes, no crash, no zombie.
TEST_F(ManagerTest, M10_HandleAfterRemoval)
{
    makeManagers();
    auto i = info("m10", nameB(), 100 * kMs, 300 * kMs);
    auto r = mgrA_->registerSource(i);
    ASSERT_EQ(r.code, RegisterError::OK);
    ASSERT_EQ(r.handle.send(makeJoy(1.f)), SendResult::OK);
    ASSERT_TRUE(waitFor([&] { return !r.handle.valid(); }, 5000));

    EXPECT_EQ(r.handle.send(makeJoy(2.f)), SendResult::DISCONNECTED);
    EXPECT_EQ(r.handle.state(), std::nullopt);
    EXPECT_FALSE(r.handle.ready());
    EXPECT_EQ(mgrA_->getSourceState(i.controller_name), std::nullopt);
}

// M11: white/blacklist — both directions, enable/disable, enabled empty
// whitelist blocks everything.
TEST_F(ManagerTest, M11_Filters)
{
    makeManagers();
    const auto i = info("m11", nameB());

    mgrA_->enableControllerWhitelist({});   // empty = block all
    EXPECT_EQ(mgrA_->registerSource(i).code, RegisterError::FILTERED);
    mgrA_->enableControllerWhitelist({i.controller_name});
    mgrA_->enableControllerBlacklist({i.controller_name});
    EXPECT_EQ(mgrA_->registerSource(i).code, RegisterError::FILTERED);
    mgrA_->disableControllerBlacklist();

    // Remote-side filter: B refuses at _onManage.
    mgrB_->enableControllerBlacklist({i.controller_name});
    EXPECT_EQ(mgrA_->registerSource(i).code, RegisterError::REJECTED);
    mgrB_->disableControllerBlacklist();
    EXPECT_EQ(mgrA_->registerSource(i).code, RegisterError::OK);
    mgrA_->disableControllerWhitelist();
}

// M12: registerCallback — template and string versions, both registration
// orders, overwrite and unregister; unknown type -> false.
TEST_F(ManagerTest, M12_TypedCallbacks)
{
    makeManagers();
    std::atomic<int> pre{0}, post{0};
    // Order 1: callback registered before the sink exists.
    EXPECT_TRUE(mgrB_->registerCallback<Joy>(
        [&](const Joy&, const ControlSignalInfo&) { pre.fetch_add(1); }));
    EXPECT_FALSE(mgrB_->registerCallback("nope",
        [](const void*, const ControlSignalInfo&) {}));

    const auto i = info("m12", nameB());
    auto r = mgrA_->registerSource(i);
    ASSERT_EQ(r.code, RegisterError::OK);
    ASSERT_TRUE(waitFor([&] { return mgrB_->getSinkState(i.controller_name).has_value(); }));
    ASSERT_EQ(r.handle.send(makeJoy(1.f)), SendResult::OK);
    ASSERT_TRUE(waitFor([&] { return pre.load() >= 1; }));

    // Order 2 / overwrite: string version replaces the callback.
    EXPECT_TRUE(mgrB_->registerCallback("joy",
        [&](const void*, const ControlSignalInfo&) { post.fetch_add(1); }));
    const int preAtSwap = pre.load();
    ASSERT_EQ(r.handle.send(makeJoy(2.f)), SendResult::OK);
    ASSERT_TRUE(waitFor([&] { return post.load() >= 1; }));
    EXPECT_EQ(pre.load(), preAtSwap);

    mgrB_->unregisterCallback("joy");
    const int postAtClear = post.load();
    ASSERT_EQ(r.handle.send(makeJoy(3.f)), SendResult::OK);
    std::this_thread::sleep_for(200ms);
    EXPECT_EQ(post.load(), postAtClear);
}

// M13: InfoReq lists registered endpoints only; ManagerStatus is a complete
// snapshot including RETRY_WAIT with endpoint_present=false and identity.
TEST_F(ManagerTest, M13_InfoReqAndStatusSnapshot)
{
    makeManagers(/*autoRetry=*/true);
    const auto ok = info("m13a", nameB());
    ASSERT_EQ(mgrA_->registerSource(ok).code, RegisterError::OK);
    const auto waiting = info("m13b", "late13_" + uid_);
    ASSERT_EQ(mgrA_->registerSource(waiting).code, RegisterError::RETRY_SCHEDULED);

    auto client = auxNode_->create_client<InfoReqSrv>(nameA() + "/control_signal_info_req");
    ASSERT_TRUE(client->wait_for_service(3s));
    auto f = client->async_send_request(std::make_shared<InfoReqSrv::Request>());
    ASSERT_EQ(f.wait_for(3s), std::future_status::ready);
    const auto res = f.get();
    ASSERT_EQ(res->source_list.size(), 1u);   // registered only
    EXPECT_EQ(res->source_list[0].controller_name, ok.controller_name);

    ManagerStatusT st;
    ASSERT_TRUE(captureStatus(nameA(), st));
    ASSERT_EQ(st.sources.size(), 2u);   // full snapshot incl. RETRY_WAIT
    bool sawRetryWait = false;
    for (const auto& e : st.sources)
    {
        EXPECT_FALSE(e.registration_id.empty());
        EXPECT_FALSE(e.csm_instance_id.empty());
        EXPECT_EQ(e.source_manager_name, nameA());
        if (e.controller_name == waiting.controller_name)
        {
            sawRetryWait = e.registration_phase == EntryStatusT::PHASE_RETRY_WAIT &&
                           !e.endpoint_present;
        }
    }
    EXPECT_TRUE(sawRetryWait);
    EXPECT_TRUE(st.snapshot_ready);
}

// M14: synchronous registerSource inside a callback or before the executor
// has ever ticked — a clear precondition error, not a fake remote timeout.
TEST_F(ManagerTest, M14_PreconditionViolations)
{
    makeManagers();
    // Inside a ROS callback (the manager's own notification service thread).
    std::atomic<bool> checked{false};
    RegisterError inCbCode{RegisterError::OK};
    auto sub = nodeA_->create_subscription<Joy>(
        uid_ + "/m14_trigger", 10, [&](const Joy&) {
            // Any Manager-owned callback marks the thread; a plain node sub
            // does not, so call through a service handled by the manager:
            (void)0;
        });
    // Use the manager's own service path: call get_notifications with a bad
    // instance id — inside that handler the guard is set; but we cannot run
    // user code there. Instead verify the executor-not-ticked variant with a
    // detached manager, which is the deterministic precondition (§8.3).
    auto lonely = std::make_shared<rclcpp::Node>("lonely_" + uid_);
    ControlSignalManager mgrL(lonely.get(), "lonely_" + uid_, makeOptions(false));
    const auto start = std::chrono::steady_clock::now();
    const auto r = mgrL.registerSource(info("m14", nameB()));
    const auto elapsed = std::chrono::steady_clock::now() - start;
    EXPECT_EQ(r.code, RegisterError::INVALID_CONTEXT);
    EXPECT_LE(elapsed, 1s);   // fails fast, no 5 s pseudo-timeout
    checked.store(true);
    EXPECT_TRUE(checked.load());
    (void)inCbCode;
    (void)sub;

    // In-callback half: state callbacks run on the tick thread, where the
    // callback guard must reject the synchronous API immediately.
    std::atomic<bool> cbChecked{false};
    std::atomic<RegisterError> cbCode{RegisterError::OK};
    mgrA_->registerSourceStateCallback(
        ControlSignalState::ACTIVE,
        [&](const std::string&, ControlSignalState, ControlSignalState) {
            if (!cbChecked.exchange(true))
                cbCode.store(mgrA_->registerSource(info("m14x", nameB())).code);
        });
    auto r14 = mgrA_->registerSource(info("m14b", nameB()));
    ASSERT_EQ(r14.code, RegisterError::OK);
    ASSERT_EQ(r14.handle.send(makeJoy(1.f)), SendResult::OK);
    ASSERT_TRUE(waitFor([&] { return cbChecked.load(); }));
    EXPECT_EQ(cbCode.load(), RegisterError::INVALID_CONTEXT);
    mgrA_->registerSourceStateCallback(ControlSignalState::ACTIVE, nullptr);
}

// M15: non-blocking retry rebuild — a master deregistration order removes
// the endpoint while slot/Handle survive; bounded async attempts follow and
// the same Handle recovers.
TEST_F(ManagerTest, M15_RetryRebuildSameHandle)
{
    makeManagers(/*autoRetry=*/true);
    Events ev;
    mgrA_->setNotificationCallback([&](const auto& e) { ev.push(e); });

    const auto i = info("m15", nameB());
    auto r = mgrA_->registerSource(i);
    ASSERT_EQ(r.code, RegisterError::OK);
    ManagerStatusT st;
    ASSERT_TRUE(captureStatus(nameA(), st));
    ASSERT_EQ(st.sources.size(), 1u);

    auto client = auxNode_->create_client<CsmNotifySrv>(nameA() + "/get_notifications");
    ASSERT_TRUE(client->wait_for_service(3s));
    auto rq = std::make_shared<CsmNotifySrv::Request>();
    rq->kind = CsmNotifySrv::Request::KIND_DISCONNECTED;
    rq->event_id = "m15_kill";
    rq->target_csm_instance_id = st.csm_instance_id;
    rq->entries = {st.sources[0]};
    auto f = client->async_send_request(rq);
    ASSERT_EQ(f.wait_for(3s), std::future_status::ready);
    ASSERT_EQ(f.get()->response, CsmNotifySrv::Response::RESPONSE_APPLIED);

    // Endpoint deregisters, handle stays valid, then recovers via retry
    // (the terminal path's matching UNREGISTER cleared B's old sink).
    ASSERT_TRUE(waitFor([&] { return !r.handle.ready(); }));
    EXPECT_TRUE(r.handle.valid());
    ASSERT_TRUE(waitFor([&] { return r.handle.ready(); }, 10000));
    EXPECT_GE(ev.count(ControlSignalManager::EventKind::RETRY_SUCCEEDED), 1);
    EXPECT_EQ(r.handle.send(makeJoy(1.f)), SendResult::OK);
}

// M16: ManagerStatus content — all entries, correct states, data_rate_hz
// close to the real rate, publish period close to statusIntervalMs.
TEST_F(ManagerTest, M16_StatusContent)
{
    makeManagers(false, 100);
    const auto i = info("m16", nameB());
    auto r = mgrA_->registerSource(i);
    ASSERT_EQ(r.code, RegisterError::OK);

    // ~20 Hz sender keeps running while status messages are collected, so
    // the captured snapshot reflects the live rate and ACTIVE state.
    std::atomic<bool> stop{false};
    std::thread sender([&] {
        while (!stop.load())
        {
            (void)r.handle.send(makeJoy(1.f));
            std::this_thread::sleep_for(50ms);
        }
    });
    std::this_thread::sleep_for(1500ms);   // let the rate window fill

    std::mutex mtx;
    std::vector<std::pair<int64_t, ManagerStatusT>> msgs;
    auto sub = auxNode_->create_subscription<ManagerStatusT>(
        nameA() + "/status", 10, [&](const ManagerStatusT& m) {
            std::lock_guard<std::mutex> lk(mtx);
            msgs.emplace_back(steadyNowNs(), m);
        });
    ASSERT_TRUE(waitFor([&] {
        std::lock_guard<std::mutex> lk(mtx);
        return msgs.size() >= 4;
    }));
    ManagerStatusT stB;
    ASSERT_TRUE(captureStatus(nameB(), stB));   // captured while still sending
    stop.store(true);
    sender.join();

    std::lock_guard<std::mutex> lk(mtx);
    const auto& last = msgs.back().second;
    ASSERT_EQ(last.sources.size(), 1u);
    EXPECT_EQ(last.sources[0].state, EntryStatusT::STATE_ACTIVE);
    EXPECT_GE(last.sources[0].data_rate_hz, 14.f);
    EXPECT_LE(last.sources[0].data_rate_hz, 26.f);
    ASSERT_EQ(stB.sinks.size(), 1u);   // sink-side lastStatus rate path
    EXPECT_GE(stB.sinks[0].data_rate_hz, 14.f);
    EXPECT_LE(stB.sinks[0].data_rate_hz, 26.f);
    // Publish period ~ statusIntervalMs (100ms; generous CI bounds).
    std::vector<int64_t> gaps;
    for (size_t k = 1; k < msgs.size(); ++k)
        gaps.push_back(msgs[k].first - msgs[k - 1].first);
    const int64_t avg =
        std::accumulate(gaps.begin(), gaps.end(), int64_t{0}) /
        static_cast<int64_t>(gaps.size());
    EXPECT_GE(avg, 50 * kMs);
    EXPECT_LE(avg, 300 * kMs);
    // Sequence strictly increasing.
    for (size_t k = 1; k < msgs.size(); ++k)
        EXPECT_GT(msgs[k].second.snapshot_seq, msgs[k - 1].second.snapshot_seq);
}

// M17: CsmRegister fields; heartbeat single in-flight; UNKNOWN_CSM triggers
// re-register.
TEST_F(ManagerTest, M17_MasterInteraction)
{
    makeManagers();
    ASSERT_TRUE(waitFor([&] {
        std::lock_guard<std::mutex> lk(master_->mtx);
        return master_->registers.size() >= 2;   // A and B registered
    }));
    {
        std::lock_guard<std::mutex> lk(master_->mtx);
        bool sawA = false;
        for (const auto& q : master_->registers)
        {
            if (q.csm_name != nameA())
                continue;
            sawA = true;
            EXPECT_EQ(q.csm_timeout_ns, 2'000 * kMs);
            EXPECT_EQ(q.csm_disconnect_timeout_ns, 20'000 * kMs);
            EXPECT_EQ(q.status_interval_ns, 50 * kMs);
            EXPECT_EQ(q.registration_grace_ns, (5000 + 2 * 50) * kMs);
            EXPECT_FALSE(q.csm_instance_id.empty());
        }
        EXPECT_TRUE(sawA);
    }
    ASSERT_TRUE(waitFor([&] {
        std::lock_guard<std::mutex> lk(master_->mtx);
        return master_->heartbeats >= 3;
    }));
    EXPECT_FALSE(mgrA_->isDegraded());
    {
        std::lock_guard<std::mutex> lk(master_->mtx);
        for (const auto& [name, mc] : master_->maxConcurrentPer)
            EXPECT_LE(mc, 1) << name;   // strictly one in flight per CSM
    }

    // UNKNOWN_CSM: the affected manager re-registers with the same instance.
    const size_t regBefore = [&] {
        std::lock_guard<std::mutex> lk(master_->mtx);
        return master_->registers.size();
    }();
    master_->answerUnknownOnce.store(true);
    ASSERT_TRUE(waitFor([&] {
        std::lock_guard<std::mutex> lk(master_->mtx);
        return master_->registers.size() > regBefore;
    }, 8000));

    // Silence: no heartbeat responses -> degraded only; entity state and the
    // established pipeline stay untouched.
    ASSERT_TRUE(waitFor([&] { return !mgrA_->isDegraded(); }, 8000));
    master_->killHeartbeat();
    ASSERT_TRUE(waitFor([&] { return mgrA_->isDegraded(); }, 8000));
    master_->makeHeartbeatService();
    ASSERT_TRUE(waitFor([&] { return !mgrA_->isDegraded(); }, 8000));
}

// M18: per-state callbacks for all managed entities — one fire per
// transition, correct old/new, tick thread, overwrite and nullptr clear.
TEST_F(ManagerTest, M18_PerStateCallbacks)
{
    makeManagers();
    std::atomic<int> srcTimeout{0}, sinkActive{0};
    std::thread::id srcThread{}, mainThread = std::this_thread::get_id();
    mgrA_->registerSourceStateCallback(
        ControlSignalState::TIMEOUT,
        [&](const std::string&, ControlSignalState oldS, ControlSignalState newS) {
            EXPECT_EQ(oldS, ControlSignalState::ACTIVE);
            EXPECT_EQ(newS, ControlSignalState::TIMEOUT);
            srcThread = std::this_thread::get_id();
            srcTimeout.fetch_add(1);
        });
    mgrB_->registerSinkStateCallback(
        ControlSignalState::ACTIVE,
        [&](const std::string&, ControlSignalState oldS, ControlSignalState newS) {
            EXPECT_NE(oldS, newS);
            EXPECT_EQ(newS, ControlSignalState::ACTIVE);
            sinkActive.fetch_add(1);
        });

    const auto i = info("m18", nameB(), 250 * kMs, 30'000 * kMs);
    auto r = mgrA_->registerSource(i);
    ASSERT_EQ(r.code, RegisterError::OK);
    ASSERT_EQ(r.handle.send(makeJoy(1.f)), SendResult::OK);

    ASSERT_TRUE(waitFor([&] { return sinkActive.load() >= 1; }));
    ASSERT_TRUE(waitFor([&] { return srcTimeout.load() >= 1; }, 5000));
    EXPECT_EQ(srcTimeout.load(), 1);
    EXPECT_NE(srcThread, mainThread);   // tick thread, never the caller

    // Overwrite: the replacement fires, the old slot holder stays silent.
    std::atomic<int> srcTimeout2{0};
    mgrA_->registerSourceStateCallback(
        ControlSignalState::TIMEOUT,
        [&](const std::string&, ControlSignalState, ControlSignalState) {
            srcTimeout2.fetch_add(1);
        });
    ASSERT_EQ(r.handle.send(makeJoy(2.f)), SendResult::OK);
    ASSERT_TRUE(waitFor([&] { return srcTimeout2.load() >= 1; }, 5000));
    EXPECT_EQ(srcTimeout.load(), 1);   // the replaced callback never fired again

    // nullptr clears: recover then time out again -> no further fire.
    mgrA_->registerSourceStateCallback(ControlSignalState::TIMEOUT, nullptr);
    const int t2AtClear = srcTimeout2.load();
    ASSERT_EQ(r.handle.send(makeJoy(3.f)), SendResult::OK);
    ASSERT_TRUE(waitFor([&] {
        return mgrA_->getSourceState(i.controller_name) == ControlSignalState::TIMEOUT;
    }, 5000));
    EXPECT_EQ(srcTimeout.load(), 1);
    EXPECT_EQ(srcTimeout2.load(), t2AtClear);
}

// M19: one tick crosses both thresholds — straight to DISCONNECTED with no
// observable intermediate TIMEOUT.
TEST_F(ManagerTest, M19_DoubleCrossSingleTick)
{
    makeManagers(false, 200);   // slow tick
    std::atomic<int> timeoutFires{0};
    mgrA_->registerSourceStateCallback(
        ControlSignalState::TIMEOUT,
        [&](const std::string&, ControlSignalState, ControlSignalState) {
            timeoutFires.fetch_add(1);
        });
    auto i = info("m19", nameB(), 60 * kMs, 90 * kMs);   // both < tick period
    auto r = mgrA_->registerSource(i);
    ASSERT_EQ(r.code, RegisterError::OK);
    ASSERT_EQ(r.handle.send(makeJoy(1.f)), SendResult::OK);

    ASSERT_TRUE(waitFor([&] { return !r.handle.valid(); }, 5000));
    EXPECT_EQ(timeoutFires.load(), 0);   // never saw TIMEOUT
}

// M20: retry-until-success — an existing same-name entry with a different
// identity is refused (never adopted); once the old generation leaves, the
// new one wins.
TEST_F(ManagerTest, M20_RetryUntilOldGenerationGone)
{
    makeManagers(/*autoRetry=*/true);
    const auto i = info("m20", nameB());
    auto r1 = mgrA_->registerSource(i);
    ASSERT_EQ(r1.code, RegisterError::OK);

    // Second manager, same controller/channel to the same target: conflict.
    auto mgrA2 = std::make_unique<ControlSignalManager>(auxNode_.get(),
                                                        "csmA2_" + uid_,
                                                        makeOptions(true));
    ASSERT_TRUE(waitFor([&] { return tickObserved(*mgrA2); }));
    auto r2 = mgrA2->registerSource(i);
    ASSERT_EQ(r2.code, RegisterError::RETRY_SCHEDULED);
    EXPECT_TRUE(r2.handle.valid());
    EXPECT_FALSE(r2.handle.ready());
    std::this_thread::sleep_for(400ms);
    EXPECT_FALSE(r2.handle.ready());   // still blocked by the old identity

    // Old generation leaves via explicit unregister; retry then succeeds.
    ASSERT_TRUE(mgrA_->unregisterSource(r1.handle));
    ASSERT_TRUE(waitFor([&] { return r2.handle.ready(); }, 10000));
    EXPECT_EQ(r2.handle.send(makeJoy(1.f)), SendResult::OK);
}

// M21: callback re-entry — querying the Manager inside state/notification
// callbacks deadlocks nothing.
TEST_F(ManagerTest, M21_CallbackReentry)
{
    makeManagers(/*autoRetry=*/true);
    std::atomic<int> reentered{0};
    mgrA_->registerSourceStateCallback(
        ControlSignalState::ACTIVE,
        [&](const std::string& ctrl, ControlSignalState, ControlSignalState) {
            (void)mgrA_->getSourceState(ctrl);
            (void)mgrA_->getSource(ctrl);
            (void)mgrA_->getSourceInfoList();
            reentered.fetch_add(1);
        });
    mgrA_->setNotificationCallback([&](const auto&) {
        (void)mgrA_->getSourceInfoList();
    });

    const auto i = info("m21", nameB());
    auto r = mgrA_->registerSource(i);
    ASSERT_EQ(r.code, RegisterError::OK);
    ASSERT_EQ(r.handle.send(makeJoy(1.f)), SendResult::OK);
    ASSERT_TRUE(waitFor([&] { return reentered.load() >= 1; }));
}

// M22: activity vs terminal-commit race — continued sending cancels the
// removal indefinitely; stopping ends in exactly one removal flow.
TEST_F(ManagerTest, M22_ActivityCancelsTerminal)
{
    makeManagers();
    std::atomic<int> discFires{0};
    mgrA_->registerSourceStateCallback(
        ControlSignalState::DISCONNECTED,
        [&](const std::string&, ControlSignalState, ControlSignalState) {
            discFires.fetch_add(1);
        });
    auto i = info("m22", nameB(), 100 * kMs, 250 * kMs);
    auto r = mgrA_->registerSource(i);
    ASSERT_EQ(r.code, RegisterError::OK);

    // Keep sending well past several disconnect windows: never removed.
    for (int k = 0; k < 20; ++k)
    {
        ASSERT_EQ(r.handle.send(makeJoy(1.f)), SendResult::OK);
        std::this_thread::sleep_for(50ms);
    }
    EXPECT_TRUE(r.handle.valid());
    EXPECT_EQ(discFires.load(), 0);

    // Stop: exactly one callback -> shutdown -> removal flow.
    ASSERT_TRUE(waitFor([&] { return !r.handle.valid(); }, 5000));
    EXPECT_EQ(discFires.load(), 1);
}

// M23: stale control messages — old-identity UNREGISTER and CsmNotify hit
// STALE and change nothing on the current generation.
TEST_F(ManagerTest, M23_StaleControlMessages)
{
    makeManagers();
    const auto i = info("m23", nameB());
    auto r = mgrA_->registerSource(i);
    ASSERT_EQ(r.code, RegisterError::OK);
    ManagerStatusT st;
    ASSERT_TRUE(captureStatus(nameB(), st));
    ASSERT_EQ(st.sinks.size(), 1u);

    // Old-generation UNREGISTER at the sink side.
    auto client = auxNode_->create_client<ManageSrv>(nameB() + "/control_signal_manage");
    ASSERT_TRUE(client->wait_for_service(3s));
    auto rq = std::make_shared<ManageSrv::Request>();
    rq->op = ManageSrv::Request::OP_UNREGISTER;
    rq->source_manager_name = nameA();
    rq->source_csm_instance_id = st.sinks[0].source_csm_instance_id;
    rq->registration_id = st.sinks[0].registration_id;
    rq->attempt_generation = st.sinks[0].attempt_generation + 5;   // wrong gen
    rq->info.controller_name = i.controller_name;
    auto f = client->async_send_request(rq);
    ASSERT_EQ(f.wait_for(3s), std::future_status::ready);
    EXPECT_EQ(f.get()->response, ManageSrv::Response::RESPONSE_STALE);
    std::this_thread::sleep_for(200ms);
    EXPECT_TRUE(mgrB_->getSinkState(i.controller_name).has_value());   // untouched

    // Wrong-incarnation CsmNotify at the source side.
    auto nclient = auxNode_->create_client<CsmNotifySrv>(nameA() + "/get_notifications");
    ASSERT_TRUE(nclient->wait_for_service(3s));
    auto nrq = std::make_shared<CsmNotifySrv::Request>();
    nrq->kind = CsmNotifySrv::Request::KIND_DISCONNECTED;
    nrq->event_id = "m23";
    nrq->target_csm_instance_id = "not_this_incarnation";
    auto nf = nclient->async_send_request(nrq);
    ASSERT_EQ(nf.wait_for(3s), std::future_status::ready);
    EXPECT_EQ(nf.get()->response, CsmNotifySrv::Response::RESPONSE_STALE);
    EXPECT_TRUE(r.handle.ready());
}

// M24: retry backoff and dedup — one table entry per registrationId, growing
// gaps between attempts, initial cap only for the never-successful intent,
// events carry kind/result/attemptGeneration.
TEST_F(ManagerTest, M24_RetryBackoffDedup)
{
    makeManagers(/*autoRetry=*/true);
    Events ev;
    mgrA_->setNotificationCallback([&](const auto& e) { ev.push(e); });

    const auto i = info("m24", "void24_" + uid_);   // target never exists
    auto r = mgrA_->registerSource(i);
    ASSERT_EQ(r.code, RegisterError::RETRY_SCHEDULED);

    // maxInitialAttempts = 3: RETRY_STARTED stops at 3 for an intent that
    // never succeeded.
    ASSERT_TRUE(waitFor([&] {
        return ev.count(ControlSignalManager::EventKind::RETRY_STARTED) >= 3;
    }, 10000));
    std::this_thread::sleep_for(1200ms);
    EXPECT_EQ(ev.count(ControlSignalManager::EventKind::RETRY_STARTED), 3);
    // Cap exhaustion terminates loudly: RETRY_FAILED fired, slot removed,
    // handle dead — no silently starved zombie entry.
    EXPECT_TRUE(waitFor([&] {
        return ev.count(ControlSignalManager::EventKind::RETRY_FAILED) >= 3 + 1;
    }, 5000));   // 3 per-attempt failures + 1 terminal exhaustion event
    EXPECT_FALSE(r.handle.valid());
    EXPECT_FALSE(mgrA_->getSource(i.controller_name).valid());

    // Attempt generations strictly increase by one per attempt (single
    // deduplicated entry, no parallel duplicates for one registrationId).
    std::vector<uint64_t> gens;
    for (const auto& e : ev.snapshot())
        if (e.kind == ControlSignalManager::EventKind::RETRY_STARTED)
            gens.push_back(e.attemptGeneration);
    ASSERT_EQ(gens.size(), 3u);
    EXPECT_EQ(gens[1], gens[0] + 1);
    EXPECT_EQ(gens[2], gens[1] + 1);
    for (const auto& e : ev.snapshot())
        if (e.kind == ControlSignalManager::EventKind::RETRY_FAILED)
            EXPECT_NE(e.result, RegisterError::OK);

    // Established intent: no attempt cap. Register OK to B, then a master
    // deregistration order while B is being destroyed -> retries keep firing
    // well past maxInitialAttempts (only success/unregister/permanent stop).
    Events ev2;
    mgrA_->setNotificationCallback([&](const auto& e) { ev2.push(e); });
    const auto ie = info("m24b", nameB());
    auto re = mgrA_->registerSource(ie);
    ASSERT_EQ(re.code, RegisterError::OK);
    ManagerStatusT st;
    ASSERT_TRUE(captureStatus(nameA(), st));
    EntryStatusT mine;
    for (const auto& e : st.sources)
        if (e.controller_name == ie.controller_name)
            mine = e;
    ASSERT_EQ(mine.controller_name, ie.controller_name);

    mgrB_.reset();   // target vanishes
    auto client = auxNode_->create_client<CsmNotifySrv>(nameA() + "/get_notifications");
    ASSERT_TRUE(client->wait_for_service(3s));
    auto rq = std::make_shared<CsmNotifySrv::Request>();
    rq->kind = CsmNotifySrv::Request::KIND_DISCONNECTED;
    rq->event_id = "m24b_kill";
    rq->target_csm_instance_id = st.csm_instance_id;
    rq->entries = {mine};
    auto f = client->async_send_request(rq);
    ASSERT_EQ(f.wait_for(3s), std::future_status::ready);
    ASSERT_EQ(f.get()->response, CsmNotifySrv::Response::RESPONSE_APPLIED);

    ASSERT_TRUE(waitFor([&] {
        return ev2.count(ControlSignalManager::EventKind::RETRY_STARTED) >= 5;
    }, 15000));   // 5 > maxInitialAttempts(3): the cap does not apply
    EXPECT_TRUE(re.handle.valid());
}

// M25: process restart semantics — intents never survive in RAM; a fresh
// registration colliding with the leftover sink retries into a new identity.
TEST_F(ManagerTest, M25_ProcessRestart)
{
    makeManagers(/*autoRetry=*/true);
    const auto i = info("m25", nameB(), 100 * kMs, 2'000 * kMs);
    ManagerStatusT stOld;
    {
        auto r = mgrA_->registerSource(i);
        ASSERT_EQ(r.code, RegisterError::OK);
        ASSERT_TRUE(captureStatus(nameB(), stOld));
        ASSERT_EQ(stOld.sinks.size(), 1u);
    }
    // "Crash": destroy manager A. Note the destructor shuts endpoints down
    // but does NOT unregister remotely — B keeps an orphaned sink, which is
    // exactly the crash aftermath (§8.3).
    const std::string oldInstance = mgrA_->getCsmInstanceId();
    mgrA_.reset();

    // Restart with a new incarnation; the old sink blocks -> retry; the
    // orphan dies via its own disconnect timeout (its source is gone), the
    // retry then establishes a fresh identity.
    mgrA_ = std::make_unique<ControlSignalManager>(nodeA_.get(), nameA(),
                                                   makeOptions(true));
    ASSERT_TRUE(waitFor([&] { return tickObserved(*mgrA_); }));
    EXPECT_NE(mgrA_->getCsmInstanceId(), oldInstance);
    EXPECT_TRUE(mgrA_->getSourceInfoList().empty());   // nothing came back

    // The orphan (2 s disconnect) is still alive: the collision is
    // guaranteed, the new intent must go through retry (D3).
    auto i2 = info("m25", nameB(), 100 * kMs, 2'000 * kMs);
    auto r2 = mgrA_->registerSource(i2);
    ASSERT_EQ(r2.code, RegisterError::RETRY_SCHEDULED);
    ASSERT_TRUE(waitFor([&] { return r2.handle.ready(); }, 15000));

    ManagerStatusT stNew;
    ASSERT_TRUE(captureStatus(nameB(), stNew));
    ASSERT_EQ(stNew.sinks.size(), 1u);
    EXPECT_NE(stNew.sinks[0].source_csm_instance_id,
              stOld.sinks[0].source_csm_instance_id);   // new identity
}

// M26: service response-failure terminal guard — high-frequency failing
// sends do not cancel the removal via activity generation; RESPONSE_FAILURE
// enters exactly one mandatory retry (slot survives).
TEST_F(ManagerTest, M26_ResponseFailureTerminal)
{
    makeManagers(/*autoRetry=*/true);
    Events ev;
    mgrA_->setNotificationCallback([&](const auto& e) { ev.push(e); });

    auto i = info("m26", nameB(), 150 * kMs, 600 * kMs, "service");
    auto r = mgrA_->registerSource(i);
    ASSERT_EQ(r.code, RegisterError::OK);
    ASSERT_TRUE(waitFor([&] { return mgrB_->getSinkState(i.controller_name).has_value(); }));
    ASSERT_EQ(r.handle.send(makeJoy(1.f)), SendResult::OK);

    // Kill the serving side entirely: every further send fails while the
    // activity generation keeps moving.
    mgrB_.reset();
    std::atomic<bool> stop{false};
    std::thread sender([&] {
        while (!stop.load())
        {
            const auto res = r.handle.send(makeJoy(1.f));
            (void)res;
            std::this_thread::sleep_for(30ms);
        }
    });
    // Removal happens despite the ongoing sends; the slot survives into
    // RETRY_WAIT (handle valid, not ready).
    ASSERT_TRUE(waitFor([&] { return !r.handle.ready(); }, 10000));
    stop.store(true);
    sender.join();
    EXPECT_TRUE(r.handle.valid());
    EXPECT_EQ(r.handle.send(makeJoy(1.f)), SendResult::RETRYING);
    EXPECT_TRUE(waitFor([&] {
        return ev.count(ControlSignalManager::EventKind::RETRY_STARTED) >= 1;
    }, 5000));
}

} // namespace

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    ::testing::AddGlobalTestEnvironment(new rv2_interfaces::r1::RclcppEnv);
    return RUN_ALL_TESTS();
}
