/**
 * @file test_handles.cpp
 * @brief H1-H8 unit tests for r1::SourceHandle / r1::SinkHandle (design
 *        §10.4). Twin managers + mock master client, same scaffolding shape
 *        as test_manager.cpp.
 */

#include <atomic>
#include <chrono>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
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
using rv2_interfaces::r1::SinkHandle;
using rv2_interfaces::r1::SourceHandle;
using Joy = sensor_msgs::msg::Joy;
using Twist = geometry_msgs::msg::Twist;
using CsmNotifySrv = r1_interfaces::srv::CsmNotify;
using CsmRegisterSrv = r1_interfaces::srv::CsmRegister;
using CsmHeartbeatSrv = r1_interfaces::srv::CsmHeartbeat;
using ManagerStatusT = r1_interfaces::msg::ManagerStatus;

constexpr int64_t kMs = 1'000'000;

bool waitFor(const std::function<bool()>& cond, int64_t timeoutMs = 5000)
{
    const auto deadline =
        std::chrono::steady_clock::now() + std::chrono::milliseconds(timeoutMs);
    while (std::chrono::steady_clock::now() < deadline)
    {
        if (cond())
            return true;
        std::this_thread::sleep_for(10ms);
    }
    return cond();
}

Joy makeJoy(float a)
{
    Joy j;
    j.axes = {a};
    return j;
}

class HandlesTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        uid_ = "h" + std::to_string(counter_++);
        nodeA_ = std::make_shared<rclcpp::Node>("h_a_" + uid_);
        nodeB_ = std::make_shared<rclcpp::Node>("h_b_" + uid_);
        auxNode_ = std::make_shared<rclcpp::Node>("h_aux_" + uid_);
        executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>(
            rclcpp::ExecutorOptions(), 4);
        executor_->add_node(nodeA_);
        executor_->add_node(nodeB_);
        executor_->add_node(auxNode_);
        // Minimal master mock: accept register + heartbeat.
        masterReg_ = auxNode_->create_service<CsmRegisterSrv>(
            master() + "/register",
            [](const std::shared_ptr<CsmRegisterSrv::Request>,
               std::shared_ptr<CsmRegisterSrv::Response> rs) {
                rs->response = CsmRegisterSrv::Response::RESPONSE_SUCCESS;
            });
        masterHb_ = auxNode_->create_service<CsmHeartbeatSrv>(
            master() + "/heartbeat",
            [](const std::shared_ptr<CsmHeartbeatSrv::Request>,
               std::shared_ptr<CsmHeartbeatSrv::Response> rs) {
                rs->response = CsmHeartbeatSrv::Response::RESPONSE_SUCCESS;
            });
        spin_ = std::thread([this] { executor_->spin(); });
        while (!executor_->is_spinning())
            std::this_thread::sleep_for(1ms);

        ManagerOptions optA = makeOptions();
        ManagerOptions optB = makeOptions();
        mgrA_ = std::make_unique<ControlSignalManager>(nodeA_.get(), nameA(), optA);
        mgrB_ = std::make_unique<ControlSignalManager>(nodeB_.get(), nameB(), optB);
        ASSERT_TRUE(waitFor([&] {
            return mgrA_->registerSource(ControlSignalInfo(), 1).code !=
                       RegisterError::INVALID_CONTEXT &&
                   mgrB_->registerSource(ControlSignalInfo(), 1).code !=
                       RegisterError::INVALID_CONTEXT;
        }));
    }

    void TearDown() override
    {
        mgrA_.reset();
        mgrB_.reset();
        executor_->cancel();
        if (spin_.joinable())
            spin_.join();
        executor_.reset();
        auxNode_.reset();
        nodeB_.reset();
        nodeA_.reset();
    }

    std::string master() const { return "hmaster_" + uid_; }
    std::string nameA() const { return "hcsmA_" + uid_; }
    std::string nameB() const { return "hcsmB_" + uid_; }

    ManagerOptions makeOptions()
    {
        RetryPolicy p(100, 800, 0.1, 3, 4);
        ManagerOptions o(p);
        o.statusIntervalMs = 50;
        o.pendingTtlMs = 800;
        o.maxRegisterTimeoutMs = 5000;
        o.masterName = master();
        o.csmTimeoutNs = 2'000 * kMs;
        o.csmDisconnectTimeoutNs = 20'000 * kMs;
        return o;
    }

    ControlSignalInfo info(const std::string& tag,
                           int64_t timeoutNs = 200 * kMs,
                           int64_t disconnectNs = 5'000 * kMs)
    {
        ControlSignalInfo i;
        i.controller_name = "ctrl_" + uid_ + "_" + tag;
        i.channel_name = uid_ + "/" + tag;
        i.target_manager_name = nameB();
        i.mode = ControlSignalInfo::MODE_TOPIC;
        i.type = "joy";
        i.priority = 50;
        i.timeout_ns = timeoutNs;
        i.disconnect_timeout_ns = disconnectNs;
        return i;
    }

    static int counter_;
    std::string uid_;
    rclcpp::Node::SharedPtr nodeA_, nodeB_, auxNode_;
    std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
    std::thread spin_;
    rclcpp::Service<CsmRegisterSrv>::SharedPtr masterReg_;
    rclcpp::Service<CsmHeartbeatSrv>::SharedPtr masterHb_;
    std::unique_ptr<ControlSignalManager> mgrA_, mgrB_;
};
int HandlesTest::counter_ = 0;

// H1: empty handles — everything is inert.
TEST_F(HandlesTest, H1_EmptyHandles)
{
    SourceHandle src;
    EXPECT_FALSE(src.valid());
    EXPECT_FALSE(src.ready());
    EXPECT_EQ(src.state(), std::nullopt);
    EXPECT_EQ(src.info(), std::nullopt);
    EXPECT_EQ(src.send(makeJoy(1.f)), SendResult::DISCONNECTED);

    SinkHandle sink;
    EXPECT_FALSE(sink.valid());
    EXPECT_EQ(sink.state(), std::nullopt);
    EXPECT_EQ(sink.info(), std::nullopt);
    Joy out;
    EXPECT_FALSE(sink.read(out));
    EXPECT_FALSE(sink.waitForMessage(out, 50 * kMs));
}

// H2: normal handles — Source valid + ready, send forwards; Sink read/state
// correct.
TEST_F(HandlesTest, H2_NormalHandles)
{
    const auto i = info("h2");
    auto r = mgrA_->registerSource(i);
    ASSERT_EQ(r.code, RegisterError::OK);
    EXPECT_TRUE(r.handle.valid());
    EXPECT_TRUE(r.handle.ready());
    EXPECT_EQ(r.handle.controllerName(), i.controller_name);
    ASSERT_TRUE(waitFor([&] {
        return mgrB_->getSinkState(i.controller_name).has_value();
    }));

    SinkHandle sink = mgrB_->getSink(i.controller_name);
    ASSERT_TRUE(sink.valid());

    Joy out;
    std::thread waiter([&] {
        SinkHandle s2 = sink;   // copies forward too
        Joy tmp;
        EXPECT_TRUE(s2.waitForMessage(tmp, 3'000 * kMs));
    });
    std::this_thread::sleep_for(100ms);
    EXPECT_EQ(r.handle.send(makeJoy(4.f)), SendResult::OK);
    waiter.join();

    ASSERT_TRUE(waitFor([&] {
        return sink.state() == ControlSignalState::ACTIVE;
    }));
    ASSERT_TRUE(sink.read(out));
    ASSERT_EQ(out.axes.size(), 1u);
    EXPECT_FLOAT_EQ(out.axes[0], 4.f);
    EXPECT_EQ(sink.info()->channel_name, i.channel_name);
    EXPECT_EQ(r.handle.state(), ControlSignalState::ACTIVE);
}

// H3: type-mismatched send — error code in release, assert in debug (§10.2).
TEST_F(HandlesTest, H3_TypeMismatch)
{
    const auto i = info("h3");
    auto r = mgrA_->registerSource(i);
    ASSERT_EQ(r.code, RegisterError::OK);
    Twist wrong;
#ifdef NDEBUG
    EXPECT_EQ(r.handle.send(wrong), SendResult::NO_TRANSPORT);
#else
    GTEST_FLAG_SET(death_test_style, "threadsafe");
    EXPECT_DEATH((void)r.handle.send(wrong), "mismatch");
#endif
}

// H4: after local deregistration / Manager erase every operation is inert;
// no crash.
TEST_F(HandlesTest, H4_InvalidAfterRemoval)
{
    const auto i = info("h4");
    auto r = mgrA_->registerSource(i);
    ASSERT_EQ(r.code, RegisterError::OK);
    ASSERT_TRUE(waitFor([&] {
        return mgrB_->getSinkState(i.controller_name).has_value();
    }));
    SinkHandle sink = mgrB_->getSink(i.controller_name);
    ASSERT_TRUE(sink.valid());

    ASSERT_TRUE(mgrA_->unregisterSource(r.handle));
    EXPECT_FALSE(r.handle.valid());
    EXPECT_FALSE(r.handle.ready());
    EXPECT_EQ(r.handle.state(), std::nullopt);
    EXPECT_EQ(r.handle.info(), std::nullopt);
    EXPECT_EQ(r.handle.send(makeJoy(1.f)), SendResult::DISCONNECTED);

    // The remote sink follows via the matching UNREGISTER.
    ASSERT_TRUE(waitFor([&] { return !sink.valid(); }));
    Joy out;
    EXPECT_FALSE(sink.read(out));
    EXPECT_EQ(sink.state(), std::nullopt);
    EXPECT_FALSE(sink.waitForMessage(out, 50 * kMs));
}

// H5: copies share the invalidation state.
TEST_F(HandlesTest, H5_CopiesShareInvalidation)
{
    const auto i = info("h5");
    auto r = mgrA_->registerSource(i);
    ASSERT_EQ(r.code, RegisterError::OK);
    SourceHandle copy1 = r.handle;
    SourceHandle copy2 = copy1;
    EXPECT_TRUE(copy1.valid());
    EXPECT_TRUE(copy2.ready());

    ASSERT_TRUE(mgrA_->unregisterSource(copy1));   // unregister via a copy
    EXPECT_FALSE(r.handle.valid());
    EXPECT_FALSE(copy1.valid());
    EXPECT_FALSE(copy2.valid());
    EXPECT_EQ(copy2.send(makeJoy(1.f)), SendResult::DISCONNECTED);
}

// H6: concurrent handle operations vs Manager removal — no UAF (full
// sanitizer coverage lands in the T12 ASan/TSan jobs).
TEST_F(HandlesTest, H6_ConcurrentOpsVsRemoval)
{
    for (int round = 0; round < 5; ++round)
    {
        auto i = info("h6_" + std::to_string(round), 100 * kMs, 60'000 * kMs);
        auto r = mgrA_->registerSource(i);
        ASSERT_EQ(r.code, RegisterError::OK);

        std::atomic<bool> stop{false};
        std::thread hammer([&] {
            Joy out;
            while (!stop.load())
            {
                (void)r.handle.send(makeJoy(1.f));
                (void)r.handle.ready();
                (void)r.handle.state();
                (void)r.handle.info();
            }
        });
        std::this_thread::sleep_for(50ms);
        ASSERT_TRUE(mgrA_->unregisterSource(r.handle));
        std::this_thread::sleep_for(50ms);
        stop.store(true);
        hammer.join();
        EXPECT_FALSE(r.handle.valid());
    }
}

// H7: peer failure -> RETRY_WAIT -> success. Same SourceHandle: valid stays
// true throughout, ready goes true -> false -> true, send returns RETRYING
// while waiting, and the new endpoint serves the same handle afterwards.
TEST_F(HandlesTest, H7_RetryKeepsHandle)
{
    const auto i = info("h7");
    auto r = mgrA_->registerSource(i);
    ASSERT_EQ(r.code, RegisterError::OK);
    ASSERT_TRUE(r.handle.ready());

    // Capture identity from A's status snapshot for the lifecycle command.
    std::mutex mtx;
    std::optional<ManagerStatusT> st;
    auto sub = auxNode_->create_subscription<ManagerStatusT>(
        nameA() + "/status", 10, [&](const ManagerStatusT& m) {
            std::lock_guard<std::mutex> lk(mtx);
            st = m;
        });
    ASSERT_TRUE(waitFor([&] {
        std::lock_guard<std::mutex> lk(mtx);
        return st && !st->sources.empty();
    }));

    auto client = auxNode_->create_client<CsmNotifySrv>(nameA() + "/get_notifications");
    ASSERT_TRUE(client->wait_for_service(3s));
    auto rq = std::make_shared<CsmNotifySrv::Request>();
    {
        std::lock_guard<std::mutex> lk(mtx);
        rq->kind = CsmNotifySrv::Request::KIND_DISCONNECTED;
        rq->event_id = "h7_kill";
        rq->target_csm_instance_id = st->csm_instance_id;
        rq->entries = {st->sources[0]};
    }
    auto f = client->async_send_request(rq);
    ASSERT_EQ(f.wait_for(3s), std::future_status::ready);
    ASSERT_EQ(f.get()->response, CsmNotifySrv::Response::RESPONSE_APPLIED);

    // Phase 2: endpoint gone, intent alive.
    ASSERT_TRUE(waitFor([&] { return !r.handle.ready(); }));
    EXPECT_TRUE(r.handle.valid());
    EXPECT_EQ(r.handle.state(), std::nullopt);      // no endpoint -> nullopt
    EXPECT_TRUE(r.handle.info().has_value());       // intent still readable
    EXPECT_EQ(r.handle.send(makeJoy(1.f)), SendResult::RETRYING);

    // Phase 3: retry succeeds (terminal path cleared B's old sink), the SAME
    // handle turns ready and sends through the replaced endpoint.
    ASSERT_TRUE(waitFor([&] { return r.handle.ready(); }, 10000));
    EXPECT_TRUE(r.handle.valid());
    EXPECT_EQ(r.handle.send(makeJoy(2.f)), SendResult::OK);
}

// H8: unregister during RETRY_WAIT — slot destroyed, handle dead, and a late
// in-flight success cannot revive it (§10.3).
TEST_F(HandlesTest, H8_UnregisterDuringRetryWait)
{
    // Register toward a target that does not exist yet: RETRY_WAIT.
    ControlSignalInfo i = info("h8");
    i.target_manager_name = "late8_" + uid_;
    auto r = mgrA_->registerSource(i);
    ASSERT_EQ(r.code, RegisterError::RETRY_SCHEDULED);
    EXPECT_TRUE(r.handle.valid());
    EXPECT_FALSE(r.handle.ready());
    EXPECT_EQ(r.handle.send(makeJoy(1.f)), SendResult::RETRYING);

    ASSERT_TRUE(mgrA_->unregisterSource(r.handle));
    EXPECT_FALSE(r.handle.valid());
    EXPECT_EQ(r.handle.send(makeJoy(1.f)), SendResult::DISCONNECTED);

    // The target shows up afterwards: nothing may revive the dead intent.
    auto late = std::make_unique<ControlSignalManager>(auxNode_.get(),
                                                       "late8_" + uid_,
                                                       makeOptions());
    std::this_thread::sleep_for(800ms);
    EXPECT_FALSE(r.handle.valid());
    EXPECT_TRUE(mgrA_->getSourceInfoList().empty());
    EXPECT_FALSE(late->getSinkState(i.controller_name).has_value());

    // Genuinely in-flight variant: the register request is outstanding on a
    // delayed-success service when the unregister lands; the late SUCCESS
    // response must not revive the slot (§10.3, H8).
    using ManageSrv = r1_interfaces::srv::ControlSignalManage;
    auto slow = auxNode_->create_service<ManageSrv>(
        "slowh8_" + uid_ + "/control_signal_manage",
        [](const std::shared_ptr<ManageSrv::Request> rq,
           std::shared_ptr<ManageSrv::Response> rs) {
            if (rq->op == ManageSrv::Request::OP_REGISTER)
                std::this_thread::sleep_for(400ms);
            rs->response = ManageSrv::Response::RESPONSE_SUCCESS;
        });
    ControlSignalInfo i2 = info("h8b");
    i2.target_manager_name = "slowh8_" + uid_;
    std::atomic<RegisterError> code{RegisterError::OK};
    std::thread reg([&] {
        code.store(mgrA_->registerSource(i2, 2000).code);
    });
    ASSERT_TRUE(waitFor([&] {
        return mgrA_->getSource(i2.controller_name).valid();
    }));
    SourceHandle pending = mgrA_->getSource(i2.controller_name);
    ASSERT_TRUE(mgrA_->unregisterSource(pending));   // while in flight
    reg.join();
    EXPECT_EQ(code.load(), RegisterError::TIMEOUT_UNKNOWN);
    EXPECT_FALSE(pending.valid());
    std::this_thread::sleep_for(600ms);   // let the late response fully land
    EXPECT_FALSE(pending.valid());        // no revival
    EXPECT_TRUE(mgrA_->getSourceInfoList().empty());
    EXPECT_FALSE(mgrA_->getSource(i2.controller_name).valid());
}

} // namespace

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    ::testing::AddGlobalTestEnvironment(new rv2_interfaces::r1::RclcppEnv);
    return RUN_ALL_TESTS();
}
