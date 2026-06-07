// ============================================================
//  test_control_signal_manager.cpp
//
//  Unit tests for ControlSignalManager (the inter-CSM registration protocol).
//
//  Migrated and expanded from the legacy rv2_server_control/test/test_csm.cpp
//  manual test (scenarios T6, T7, T9, T10) and converted to GoogleTest, with
//  additional coverage for reverse type lookup, auto-disconnect, service-mode
//  registration, and unregistered-type rejection.
// ============================================================

#include "csm_test_utils.h"

#include <atomic>

#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/string.hpp>

using namespace rv2_test;

using rv2_interfaces::ControlSignalState;
using rv2_interfaces::ControlSignalManager;

using Joy   = sensor_msgs::msg::Joy;
using Twist = geometry_msgs::msg::Twist;
using Str   = std_msgs::msg::String;
using CSConst = rv2_interfaces::msg::ControlSignalConst;

class ManagerTest : public CsmTestBase {};

// ── T6: registerSource creates the remote Sink; duplicate is rejected ─────────
TEST_F(ManagerTest, RegisterSourceCreatesRemoteSink)
{
    auto nodeA = makeNode("cm_t6_a");
    auto nodeB = makeNode("cm_t6_b");
    ControlSignalManager csmA(nodeA.get(), "cm_t6_csm_a");
    ControlSignalManager csmB(nodeB.get(), "cm_t6_csm_b");

    auto info = makeInfo("cm6/joy",
                         CSConst::CONTROL_SIGNAL_TYPE_JOY,
                         CSConst::CONTROL_SIGNAL_MODE_TOPIC,
                         "cm_t6_csm_b");

    ASSERT_TRUE(csmA.registerSource(info, 3000));
    EXPECT_NE(csmA.getSource("cm6/joy"), nullptr);
    EXPECT_NE(csmB.getSink("cm6/joy"), nullptr)
        << "remote Sink was not created on csmB";

    EXPECT_EQ(csmA.getSourceState("cm6/joy"), ControlSignalState::UNKNOWN);

    // Duplicate registration must be rejected.
    EXPECT_FALSE(csmA.registerSource(info, 3000));
}

// ── T7: control_signal_info_req lists sources / sinks correctly ───────────────
TEST_F(ManagerTest, InfoReqListsSourcesAndSinks)
{
    auto nodeC = makeNode("cm_t7_c");
    auto nodeD = makeNode("cm_t7_d");
    ControlSignalManager csmC(nodeC.get(), "cm_t7_csm_c");
    ControlSignalManager csmD(nodeD.get(), "cm_t7_csm_d");

    auto info1 = makeInfo("cm7/joy",
                          CSConst::CONTROL_SIGNAL_TYPE_JOY,
                          CSConst::CONTROL_SIGNAL_MODE_TOPIC,
                          "cm_t7_csm_d");
    auto info2 = makeInfo("cm7/twist",
                          CSConst::CONTROL_SIGNAL_TYPE_TWIST,
                          CSConst::CONTROL_SIGNAL_MODE_TOPIC,
                          "cm_t7_csm_d");

    ASSERT_TRUE(csmC.registerSource(info1, 3000));
    ASSERT_TRUE(csmC.registerSource(info2, 3000));

    auto srcList = csmC.getSourceInfoList();
    auto snkList = csmD.getSinkInfoList();
    EXPECT_EQ(srcList.size(), 2u);
    EXPECT_TRUE(csmC.getSinkInfoList().empty());
    EXPECT_EQ(snkList.size(), 2u);
    EXPECT_TRUE(csmD.getSourceInfoList().empty());
}

// ── T9: setSinkMsgCallback fires when a Sink stores a message ─────────────────
TEST_F(ManagerTest, SinkMsgCallbackFires)
{
    auto nodeE = makeNode("cm_t9_e");
    auto nodeF = makeNode("cm_t9_f");
    ControlSignalManager csmE(nodeE.get(), "cm_t9_csm_e");
    ControlSignalManager csmF(nodeF.get(), "cm_t9_csm_f");

    std::atomic<int> cbCount{0};
    std::string      cbChannel;

    // Register the callback BEFORE the source registers — verifies retroactive
    // application to the Sink created later.
    csmF.setSinkMsgCallback<Joy>(
        [&](const Joy&, const rv2_interfaces::msg::ControlSignalInfo& i)
        {
            ++cbCount;
            cbChannel = i.channel_name;
        });

    auto info = makeInfo("cm9/joy_cb",
                         CSConst::CONTROL_SIGNAL_TYPE_JOY,
                         CSConst::CONTROL_SIGNAL_MODE_TOPIC,
                         "cm_t9_csm_f");
    ASSERT_TRUE(csmE.registerSource(info, 3000));

    rclcpp::sleep_for(300ms);  // discovery

    auto src = csmE.getSource("cm9/joy_cb");
    ASSERT_NE(src, nullptr);

    Joy joy;
    joy.axes = {0.5f, -0.3f};
    bool ok = false;
    src->sendErased(&joy, ok);

    rclcpp::sleep_for(300ms);

    EXPECT_GT(cbCount.load(), 0);
    EXPECT_EQ(cbChannel, "cm9/joy_cb");
}

// ── T10: multiple types (Joy + Twist) in the same CSM pair ────────────────────
TEST_F(ManagerTest, MultipleTypesSameCsmPair)
{
    auto nodeG = makeNode("cm_t10_g");
    auto nodeH = makeNode("cm_t10_h");
    ControlSignalManager csmG(nodeG.get(), "cm_t10_csm_g");
    ControlSignalManager csmH(nodeH.get(), "cm_t10_csm_h");

    auto joyInfo = makeInfo("cm10/joy",
                            CSConst::CONTROL_SIGNAL_TYPE_JOY,
                            CSConst::CONTROL_SIGNAL_MODE_TOPIC,
                            "cm_t10_csm_h");
    auto twsInfo = makeInfo("cm10/twist",
                            CSConst::CONTROL_SIGNAL_TYPE_TWIST,
                            CSConst::CONTROL_SIGNAL_MODE_TOPIC,
                            "cm_t10_csm_h");

    ASSERT_TRUE(csmG.registerSource(joyInfo, 3000));
    ASSERT_TRUE(csmG.registerSource(twsInfo, 3000));

    ASSERT_EQ(csmG.getSourceInfoList().size(), 2u);
    ASSERT_EQ(csmH.getSinkInfoList().size(), 2u);

    ASSERT_NE(csmH.getSink("cm10/joy"), nullptr);
    ASSERT_NE(csmH.getSink("cm10/twist"), nullptr);

    // Sinks report the correct concrete message types.
    EXPECT_EQ(csmH.getSink("cm10/joy")->msgType(),   std::type_index(typeid(Joy)));
    EXPECT_EQ(csmH.getSink("cm10/twist")->msgType(), std::type_index(typeid(Twist)));

    rclcpp::sleep_for(300ms);  // discovery

    Joy joy;
    joy.axes = {1.0f};
    bool ok = false;
    csmG.getSource("cm10/joy")->sendErased(&joy, ok);
    rclcpp::sleep_for(200ms);
    EXPECT_EQ(csmH.getSinkState("cm10/joy"), ControlSignalState::ACTIVE);

    Twist tw;
    tw.linear.x = 2.0;
    csmG.getSource("cm10/twist")->sendErased(&tw, ok);
    rclcpp::sleep_for(200ms);
    EXPECT_EQ(csmH.getSinkState("cm10/twist"), ControlSignalState::ACTIVE);
}

// ── NEW: multiple sources of the SAME type on different channels ──────────────
TEST_F(ManagerTest, MultipleSourcesSameTypeDifferentChannels)
{
    auto nodeA = makeNode("cm_ms_a");
    auto nodeB = makeNode("cm_ms_b");
    ControlSignalManager csmA(nodeA.get(), "cm_ms_csm_a");
    ControlSignalManager csmB(nodeB.get(), "cm_ms_csm_b");

    auto infoA = makeInfo("cmMS/joy_a",
                          CSConst::CONTROL_SIGNAL_TYPE_JOY,
                          CSConst::CONTROL_SIGNAL_MODE_TOPIC,
                          "cm_ms_csm_b");
    auto infoB = makeInfo("cmMS/joy_b",
                          CSConst::CONTROL_SIGNAL_TYPE_JOY,
                          CSConst::CONTROL_SIGNAL_MODE_TOPIC,
                          "cm_ms_csm_b");

    ASSERT_TRUE(csmA.registerSource(infoA, 3000));
    ASSERT_TRUE(csmA.registerSource(infoB, 3000));

    ASSERT_EQ(csmB.getSinkInfoList().size(), 2u);
    ASSERT_NE(csmB.getSink("cmMS/joy_a"), nullptr);
    ASSERT_NE(csmB.getSink("cmMS/joy_b"), nullptr);

    rclcpp::sleep_for(300ms);  // discovery

    // Send only on channel A → only A's sink becomes ACTIVE.
    Joy joy;
    joy.axes = {0.9f};
    bool ok = false;
    csmA.getSource("cmMS/joy_a")->sendErased(&joy, ok);
    rclcpp::sleep_for(200ms);

    EXPECT_EQ(csmB.getSinkState("cmMS/joy_a"), ControlSignalState::ACTIVE);
    EXPECT_EQ(csmB.getSinkState("cmMS/joy_b"), ControlSignalState::UNKNOWN);
}

// ── NEW: ControlSignalManager::typeKeyFor reverse lookup ──────────────────────
TEST_F(ManagerTest, TypeKeyForReverseLookup)
{
    EXPECT_EQ(ControlSignalManager::typeKeyFor<Joy>(),
              CSConst::CONTROL_SIGNAL_TYPE_JOY);
    EXPECT_EQ(ControlSignalManager::typeKeyFor<Twist>(),
              CSConst::CONTROL_SIGNAL_TYPE_TWIST);
    EXPECT_EQ(ControlSignalManager::typeKeyFor<Str>(),
              CSConst::CONTROL_SIGNAL_TYPE_STRING);

    // A message type that was never registered resolves to "unknown".
    struct NeverRegisteredMsg {};
    EXPECT_EQ(ControlSignalManager::typeKeyFor<NeverRegisteredMsg>(),
              CSConst::CONTROL_SIGNAL_TYPE_UNKNOWN);
}

// ── NEW: auto-disconnect — a TIMED-OUT Sink is removed by the status timer ────
TEST_F(ManagerTest, AutoDisconnectAfterTimeout)
{
    auto nodeA = makeNode("cm_ad_a");
    auto nodeB = makeNode("cm_ad_b");
    // Fast status timer so the disconnect sweep runs frequently.
    ControlSignalManager csmA(nodeA.get(), "cm_ad_csm_a", 200);
    ControlSignalManager csmB(nodeB.get(), "cm_ad_csm_b", 200);

    // timeout_ns = 300 ms; disconnect_timeout_ns = 600 ms (> timeout, per Rule 6).
    auto info = makeInfo("cmAD/joy",
                         CSConst::CONTROL_SIGNAL_TYPE_JOY,
                         CSConst::CONTROL_SIGNAL_MODE_TOPIC,
                         "cm_ad_csm_b",
                         false, 0,
                         300'000'000LL,   // timeout 300 ms
                         0.0f,
                         600'000'000LL);  // disconnect 600 ms

    ASSERT_TRUE(csmA.registerSource(info, 3000));
    ASSERT_NE(csmB.getSink("cmAD/joy"), nullptr);

    rclcpp::sleep_for(300ms);  // discovery

    // Drive the Sink ACTIVE. Retry sends (each well within the 300 ms timeout)
    // until the subscription is matched, so a slow-to-discover DDS link cannot
    // make this precondition flaky.
    Joy joy;
    joy.axes = {0.5f};
    bool ok = false;
    auto src = csmA.getSource("cmAD/joy");
    ASSERT_NE(src, nullptr);
    bool becameActive = false;
    for (int i = 0; i < 20 && !becameActive; ++i)
    {
        src->sendErased(&joy, ok);
        rclcpp::sleep_for(50ms);
        becameActive = (csmB.getSinkState("cmAD/joy") == ControlSignalState::ACTIVE);
    }
    ASSERT_TRUE(becameActive);

    // Stop sending: Sink → TIMEOUT (after 300 ms) → removed (after 600 ms more).
    rclcpp::sleep_for(2000ms);

    EXPECT_EQ(csmB.getSink("cmAD/joy"), nullptr)
        << "timed-out Sink should have been auto-disconnected/removed";
}

// ── NEW: service-mode registration + delivery through the CSM ─────────────────
TEST_F(ManagerTest, ServiceModeRegistrationAndDelivery)
{
    auto nodeA = makeNode("cm_sm_a");
    auto nodeB = makeNode("cm_sm_b");
    ControlSignalManager csmA(nodeA.get(), "cm_sm_csm_a");
    ControlSignalManager csmB(nodeB.get(), "cm_sm_csm_b");

    auto info = makeInfo("cmSM/joy_svc",
                         CSConst::CONTROL_SIGNAL_TYPE_JOY,
                         CSConst::CONTROL_SIGNAL_MODE_SERVICE,
                         "cm_sm_csm_b",
                         false, 0, 500'000'000LL);

    ASSERT_TRUE(csmA.registerSource(info, 3000));
    ASSERT_NE(csmB.getSink("cmSM/joy_svc"), nullptr);

    rclcpp::sleep_for(300ms);  // service ready

    auto src = csmA.getSource("cmSM/joy_svc");
    ASSERT_NE(src, nullptr);

    Joy joy;
    joy.axes = {0.42f};
    bool ok = false;
    ASSERT_TRUE(src->sendErased(&joy, ok));
    EXPECT_TRUE(ok);

    EXPECT_EQ(csmA.getSourceState("cmSM/joy_svc"), ControlSignalState::ACTIVE);
    EXPECT_EQ(csmB.getSinkState("cmSM/joy_svc"),   ControlSignalState::ACTIVE);

    Joy out;
    ASSERT_TRUE(csmB.getSink("cmSM/joy_svc")->readErased(&out));
    ASSERT_FALSE(out.axes.empty());
    EXPECT_FLOAT_EQ(out.axes[0], 0.42f);
}

// ── NEW: an unregistered control-signal type is rejected on registration ──────
TEST_F(ManagerTest, UnregisteredTypeRejected)
{
    auto nodeA = makeNode("cm_ur_a");
    auto nodeB = makeNode("cm_ur_b");
    ControlSignalManager csmA(nodeA.get(), "cm_ur_csm_a");
    ControlSignalManager csmB(nodeB.get(), "cm_ur_csm_b");

    auto info = makeInfo("cmUR/bad",
                         "totally_unregistered_type",
                         CSConst::CONTROL_SIGNAL_MODE_TOPIC,
                         "cm_ur_csm_b");

    EXPECT_FALSE(csmA.registerSource(info, 2000));
    EXPECT_EQ(csmA.getSource("cmUR/bad"), nullptr);
    EXPECT_EQ(csmB.getSink("cmUR/bad"), nullptr);
}

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    ::testing::AddGlobalTestEnvironment(new rv2_test::RclcppEnvironment());
    return RUN_ALL_TESTS();
}
