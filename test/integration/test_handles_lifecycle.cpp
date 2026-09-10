/**
 * @file test_handles_lifecycle.cpp
 * @brief H7-H8 Handle retry lifecycle integration tests (design §10.4).
 */

#include "r1_handle_test_utils.h"

namespace
{

// Distinct prefixes keep the two executables isolated when CTest runs in parallel.
class HandlesTest : public HandleTestBase
{
public:
    HandlesTest() : HandleTestBase("hl") {}
};

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
