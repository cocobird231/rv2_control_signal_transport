/**
 * @file test_handles.cpp
 * @brief H1-H6 Handle contract unit tests (design §10.4).
 */

#include "r1_handle_test_utils.h"

namespace
{

// Distinct prefixes keep the two executables isolated when CTest runs in parallel.
class HandlesTest : public HandleTestBase
{
public:
    HandlesTest() : HandleTestBase("h") {}
};

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


} // namespace

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    ::testing::AddGlobalTestEnvironment(new rv2_interfaces::r1::RclcppEnv);
    return RUN_ALL_TESTS();
}
