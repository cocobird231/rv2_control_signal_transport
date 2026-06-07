// ============================================================
//  test_control_signal_transport.cpp
//
//  Unit tests for the transport primitives: ControlSignalSource,
//  ControlSignalSink, and the ControlSignalFactory runtime registry.
//
//  Migrated and expanded from the legacy rv2_server_control/test/test_csm.cpp
//  manual test (scenarios T1-T5, T8, T11) and converted to GoogleTest, with
//  additional coverage for the type-erased interface and the factory.
// ============================================================

#include "csm_test_utils.h"

#include <cmath>

#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/string.hpp>
#include <rv2_interfaces/srv/control_signal_joy.hpp>
#include <rv2_interfaces/srv/control_signal_twist.hpp>

#include "rv2_control_signal_transport/control_signal_factory.h"

using namespace rv2_test;

using rv2_interfaces::ControlSignalState;
using rv2_interfaces::ControlSignalSource;
using rv2_interfaces::ControlSignalSink;
using rv2_interfaces::BaseControlSignalSource;
using rv2_interfaces::BaseControlSignalSink;
using rv2_interfaces::ControlSignalFactory;

using Joy   = sensor_msgs::msg::Joy;
using Twist = geometry_msgs::msg::Twist;
using Str   = std_msgs::msg::String;
using ControlSignalJoy   = rv2_interfaces::srv::ControlSignalJoy;
using ControlSignalTwist = rv2_interfaces::srv::ControlSignalTwist;
using CSConst = rv2_interfaces::msg::ControlSignalConst;

class TransportTest : public CsmTestBase {};

// ── T1: Source topic mode — always UNKNOWN initially ──────────────────────────
TEST_F(TransportTest, SourceTopicInitialUnknown)
{
    auto node = makeNode("tt_src_topic");
    auto info = makeInfo("tt1/joy",
                         CSConst::CONTROL_SIGNAL_TYPE_JOY,
                         CSConst::CONTROL_SIGNAL_MODE_TOPIC);

    ControlSignalSource<Joy> src(node.get(), info);
    EXPECT_EQ(src.getState(), ControlSignalState::UNKNOWN)
        << "got " << stateName(src.getState());
}

// ── T2: Sink topic mode — UNKNOWN → ACTIVE on first message ───────────────────
TEST_F(TransportTest, SinkTopicActiveOnReceive)
{
    auto nodeA = makeNode("tt_sink_a");
    auto nodeB = makeNode("tt_src_b");
    auto info  = makeInfo("tt2/joy",
                          CSConst::CONTROL_SIGNAL_TYPE_JOY,
                          CSConst::CONTROL_SIGNAL_MODE_TOPIC);

    ControlSignalSink<Joy>   sink(nodeA.get(), info);
    ControlSignalSource<Joy> src (nodeB.get(), info);

    ASSERT_EQ(sink.getState(), ControlSignalState::UNKNOWN);

    rclcpp::sleep_for(300ms);  // topic discovery

    bool cmdOk = false;
    Joy joy;
    joy.axes = {0.5f, -0.5f};
    src.send(joy, cmdOk);

    rclcpp::sleep_for(200ms);  // delivery

    EXPECT_EQ(sink.getState(), ControlSignalState::ACTIVE)
        << "got " << stateName(sink.getState());

    Joy out;
    ASSERT_TRUE(sink.read(out));
    ASSERT_FALSE(out.axes.empty());
    EXPECT_FLOAT_EQ(out.axes[0], 0.5f);
}

// ── T3: Source service mode — UNKNOWN → ACTIVE after successful send() ────────
TEST_F(TransportTest, SourceServiceModeActiveAfterSend)
{
    auto nodeA = makeNode("tt_svc_sink_a");
    auto nodeB = makeNode("tt_svc_src_b");
    auto info  = makeInfo("tt3/joy_svc",
                          CSConst::CONTROL_SIGNAL_TYPE_JOY,
                          CSConst::CONTROL_SIGNAL_MODE_SERVICE,
                          "", false, 0, 500'000'000LL);

    ControlSignalSink<Joy, ControlSignalJoy>   sink(nodeA.get(), info);
    ControlSignalSource<Joy, ControlSignalJoy> src (nodeB.get(), info);

    ASSERT_EQ(src.getState(), ControlSignalState::UNKNOWN);

    rclcpp::sleep_for(300ms);  // service ready

    bool cmdOk = false;
    Joy joy;
    joy.axes = {1.0f};
    bool sent = src.send(joy, cmdOk);

    ASSERT_TRUE(sent);
    ASSERT_TRUE(cmdOk);
    EXPECT_EQ(src.getState(),  ControlSignalState::ACTIVE);
    EXPECT_EQ(sink.getState(), ControlSignalState::ACTIVE);
}

// ── T4: Sink timeout — ACTIVE → TIMEOUT after timeout_ns elapses ──────────────
TEST_F(TransportTest, SinkTimeout)
{
    auto nodeA = makeNode("tt_to_sink_a");
    auto nodeB = makeNode("tt_to_src_b");
    auto info  = makeInfo("tt4/twist",
                          CSConst::CONTROL_SIGNAL_TYPE_TWIST,
                          CSConst::CONTROL_SIGNAL_MODE_TOPIC,
                          "", false, 0, 400'000'000LL);  // 400 ms timeout

    ControlSignalSink<Twist>   sink(nodeA.get(), info);
    ControlSignalSource<Twist> src (nodeB.get(), info);

    rclcpp::sleep_for(300ms);  // discovery

    bool cmdOk = false;
    Twist twist;
    twist.linear.x = 1.0;
    src.send(twist, cmdOk);

    rclcpp::sleep_for(150ms);  // delivery
    ASSERT_EQ(sink.getState(), ControlSignalState::ACTIVE);

    rclcpp::sleep_for(600ms);  // exceed timeout
    EXPECT_EQ(sink.getState(), ControlSignalState::TIMEOUT)
        << "got " << stateName(sink.getState());
}

// ── T5: Keep-alive — Source becomes ACTIVE from the Sink's heartbeat ──────────
TEST_F(TransportTest, KeepAliveSourceActive)
{
    auto nodeA = makeNode("tt_ka_sink_a");
    auto nodeB = makeNode("tt_ka_src_b");
    auto info  = makeInfo("tt5/joy_ka",
                          CSConst::CONTROL_SIGNAL_TYPE_JOY,
                          CSConst::CONTROL_SIGNAL_MODE_TOPIC,
                          "", true, 250'000'000LL, 2'000'000'000LL);

    ControlSignalSink<Joy>   sink(nodeA.get(), info);  // heartbeat publisher
    ControlSignalSource<Joy> src (nodeB.get(), info);  // heartbeat subscriber

    ASSERT_EQ(src.getState(), ControlSignalState::UNKNOWN);

    rclcpp::sleep_for(1100ms);  // ≥3 heartbeat ticks + discovery

    EXPECT_EQ(src.getState(), ControlSignalState::ACTIVE)
        << "got " << stateName(src.getState());
}

// ── T8: Twist service mode — round-trips values correctly ─────────────────────
TEST_F(TransportTest, TwistServiceModeRoundTrip)
{
    auto nodeA = makeNode("tt_tws_sink_a");
    auto nodeB = makeNode("tt_tws_src_b");
    auto info  = makeInfo("tt8/twist_svc",
                          CSConst::CONTROL_SIGNAL_TYPE_TWIST,
                          CSConst::CONTROL_SIGNAL_MODE_SERVICE,
                          "", false, 0, 500'000'000LL);

    ControlSignalSink<Twist, ControlSignalTwist>   sink(nodeA.get(), info);
    ControlSignalSource<Twist, ControlSignalTwist> src (nodeB.get(), info);

    ASSERT_EQ(src.getState(), ControlSignalState::UNKNOWN);

    rclcpp::sleep_for(300ms);  // service ready

    bool cmdOk = false;
    Twist twist;
    twist.linear.x  = 1.5;
    twist.linear.y  = 0.25;
    twist.angular.z = 0.75;
    bool sent = src.send(twist, cmdOk);

    ASSERT_TRUE(sent);
    ASSERT_TRUE(cmdOk);
    EXPECT_EQ(src.getState(),  ControlSignalState::ACTIVE);
    EXPECT_EQ(sink.getState(), ControlSignalState::ACTIVE);

    Twist out;
    ASSERT_TRUE(sink.read(out));
    EXPECT_NEAR(out.linear.x,  1.5,  1e-6);
    EXPECT_NEAR(out.linear.y,  0.25, 1e-6);
    EXPECT_NEAR(out.angular.z, 0.75, 1e-6);
}

// ── T11: LOW_FREQ — ACTIVE → LOW_FREQ when elapsed crosses timeout/2 ───────────
// NOTE: LOW_FREQ is driven by the liveness rule `elapsed > timeout_ns / 2`
// (see ControlSignalSink::_checkTimeout). `send_freq_hz` is monitoring-only
// metadata and does NOT influence the state machine.
TEST_F(TransportTest, SinkLowFreq)
{
    auto nodeA = makeNode("tt_lf_sink_a");
    auto nodeB = makeNode("tt_lf_src_b");
    auto info  = makeInfo("tt11/joy",
                          CSConst::CONTROL_SIGNAL_TYPE_JOY,
                          CSConst::CONTROL_SIGNAL_MODE_TOPIC,
                          "", false, 0,
                          2'000'000'000LL,  // 2 s timeout → LOW_FREQ after 1 s
                          5.0f,             // monitoring-only metadata
                          0);

    ControlSignalSink<Joy>   sink(nodeA.get(), info);
    ControlSignalSource<Joy> src (nodeB.get(), info);

    ASSERT_EQ(sink.getState(), ControlSignalState::UNKNOWN);

    rclcpp::sleep_for(300ms);  // discovery

    bool cmdOk = false;
    Joy joy;
    joy.axes = {0.5f, -0.5f};
    src.send(joy, cmdOk);
    rclcpp::sleep_for(80ms);
    ASSERT_EQ(sink.getState(), ControlSignalState::ACTIVE);

    // Elapsed since last message crosses timeout/2 (1 s) but stays below timeout (2 s).
    rclcpp::sleep_for(1200ms);
    EXPECT_EQ(sink.getState(), ControlSignalState::LOW_FREQ)
        << "got " << stateName(sink.getState());

    // A fresh message restores ACTIVE.
    src.send(joy, cmdOk);
    rclcpp::sleep_for(80ms);
    EXPECT_EQ(sink.getState(), ControlSignalState::ACTIVE);

    rclcpp::sleep_for(2100ms);  // exceed full timeout
    EXPECT_EQ(sink.getState(), ControlSignalState::TIMEOUT)
        << "got " << stateName(sink.getState());
}

// ── NEW: String topic delivery ────────────────────────────────────────────────
TEST_F(TransportTest, StringTopicDelivery)
{
    auto nodeA = makeNode("tt_str_sink_a");
    auto nodeB = makeNode("tt_str_src_b");
    auto info  = makeInfo("ttN/string",
                          CSConst::CONTROL_SIGNAL_TYPE_STRING,
                          CSConst::CONTROL_SIGNAL_MODE_TOPIC);

    ControlSignalSink<Str>   sink(nodeA.get(), info);
    ControlSignalSource<Str> src (nodeB.get(), info);

    rclcpp::sleep_for(300ms);  // discovery

    bool cmdOk = false;
    Str msg;
    msg.data = "hello-rv2";
    src.send(msg, cmdOk);

    rclcpp::sleep_for(200ms);  // delivery

    EXPECT_EQ(sink.getState(), ControlSignalState::ACTIVE);
    Str out;
    ASSERT_TRUE(sink.read(out));
    EXPECT_EQ(out.data, "hello-rv2");
}

// ── NEW: Type-erased send/read through base pointers ──────────────────────────
TEST_F(TransportTest, ErasedSendAndRead)
{
    auto nodeA = makeNode("tt_er_sink_a");
    auto nodeB = makeNode("tt_er_src_b");
    auto info  = makeInfo("ttN/erased",
                          CSConst::CONTROL_SIGNAL_TYPE_JOY,
                          CSConst::CONTROL_SIGNAL_MODE_TOPIC);

    ControlSignalSink<Joy>   sink(nodeA.get(), info);
    ControlSignalSource<Joy> src (nodeB.get(), info);

    BaseControlSignalSink*   bsnk = &sink;
    BaseControlSignalSource* bsrc = &src;

    // msgType() reports the concrete message type without knowing srvT.
    EXPECT_EQ(bsrc->msgType(), std::type_index(typeid(Joy)));
    EXPECT_EQ(bsnk->msgType(), std::type_index(typeid(Joy)));

    rclcpp::sleep_for(300ms);  // discovery

    Joy joy;
    joy.axes = {0.1f, 0.2f, 0.3f};
    bool cmdOk = false;
    ASSERT_TRUE(bsrc->sendErased(&joy, cmdOk));

    rclcpp::sleep_for(200ms);  // delivery

    Joy out;
    ASSERT_TRUE(bsnk->readErased(&out));
    ASSERT_EQ(out.axes.size(), 3u);
    EXPECT_FLOAT_EQ(out.axes[2], 0.3f);
}

// ── NEW: markDisconnected() drives the terminal DISCONNECTED state ────────────
TEST_F(TransportTest, MarkDisconnected)
{
    auto node = makeNode("tt_disc");
    auto info = makeInfo("ttN/disc",
                         CSConst::CONTROL_SIGNAL_TYPE_JOY,
                         CSConst::CONTROL_SIGNAL_MODE_TOPIC);

    ControlSignalSource<Joy> src(node.get(), info);
    ControlSignalSink<Joy>   sink(node.get(), info);

    ASSERT_NE(src.getState(),  ControlSignalState::DISCONNECTED);
    ASSERT_NE(sink.getState(), ControlSignalState::DISCONNECTED);

    src.markDisconnected();
    sink.markDisconnected();

    EXPECT_EQ(src.getState(),  ControlSignalState::DISCONNECTED);
    EXPECT_EQ(sink.getState(), ControlSignalState::DISCONNECTED);
}

// ── Reverse lookup via ControlSignalFactory ────────────────────────────────────
TEST_F(TransportTest, ReverseTypeKeyLookup)
{
    auto& factory = ControlSignalFactory::Instance();
    EXPECT_EQ(factory.typeKey(std::type_index(typeid(Joy))),   CSConst::CONTROL_SIGNAL_TYPE_JOY);
    EXPECT_EQ(factory.typeKey(std::type_index(typeid(Twist))), CSConst::CONTROL_SIGNAL_TYPE_TWIST);
    EXPECT_EQ(factory.typeKey(std::type_index(typeid(Str))),   CSConst::CONTROL_SIGNAL_TYPE_STRING);

    // An unregistered type resolves to an empty string.
    struct NeverRegistered {};
    EXPECT_TRUE(factory.typeKey(std::type_index(typeid(NeverRegistered))).empty());
}

// ── Factory runtime dispatch ───────────────────────────────────────────────────
TEST_F(TransportTest, FactoryCreateSourceSink)
{
    auto node = makeNode("tt_factory");
    auto& factory = ControlSignalFactory::Instance();

    // Topic-mode joy source/sink.
    auto topicInfo = makeInfo("ttN/factory_topic",
                              CSConst::CONTROL_SIGNAL_TYPE_JOY,
                              CSConst::CONTROL_SIGNAL_MODE_TOPIC);
    auto src = factory.CreateSource(CSConst::CONTROL_SIGNAL_TYPE_JOY, node.get(), topicInfo);
    auto snk = factory.CreateSink(CSConst::CONTROL_SIGNAL_TYPE_JOY, node.get(), topicInfo);
    ASSERT_NE(src, nullptr);
    ASSERT_NE(snk, nullptr);
    EXPECT_EQ(src->msgType(), std::type_index(typeid(Joy)));
    EXPECT_EQ(snk->msgType(), std::type_index(typeid(Joy)));

    // Service-mode joy is supported (ControlSignalJoy) → non-null.
    auto svcInfo = makeInfo("ttN/factory_svc",
                            CSConst::CONTROL_SIGNAL_TYPE_JOY,
                            CSConst::CONTROL_SIGNAL_MODE_SERVICE,
                            "", false, 0, 500'000'000LL);
    EXPECT_NE(factory.CreateSource(CSConst::CONTROL_SIGNAL_TYPE_JOY, node.get(), svcInfo), nullptr);
    EXPECT_NE(factory.CreateSink(CSConst::CONTROL_SIGNAL_TYPE_JOY, node.get(), svcInfo), nullptr);

    // Unknown type → throws std::runtime_error.
    auto badInfo = makeInfo("ttN/factory_bad", "no_such_type",
                            CSConst::CONTROL_SIGNAL_MODE_TOPIC);
    EXPECT_THROW(factory.CreateSource("no_such_type", node.get(), badInfo), std::runtime_error);
    EXPECT_THROW(factory.CreateSink("no_such_type", node.get(), badInfo), std::runtime_error);
}

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    ::testing::AddGlobalTestEnvironment(new rv2_test::RclcppEnvironment());
    return RUN_ALL_TESTS();
}
