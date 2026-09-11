/**
 * @file test_info_validation.cpp
 * @brief V1-V11 unit tests for validateControlSignalInfo() (design §3.3).
 *
 * Pure logic — no rclcpp init. makeR1Info() builds a fully valid default and
 * each case breaks exactly one field ("single-field damage"), asserting
 * valid == false and that the error names the field.
 */

#include <string>

#include <gtest/gtest.h>

#include "rv2_control_signal_transport/r1/control_signal_info.h"

namespace
{

using rv2_interfaces::r1::ControlSignalInfo;
using rv2_interfaces::r1::validateControlSignalInfo;

constexpr int64_t kMs = 1'000'000;

/// Legal defaults (§3.3 builder helper).
ControlSignalInfo makeR1Info()
{
    ControlSignalInfo info;
    info.controller_name = "ctrl_a";
    info.channel_name = "chan/a";
    info.target_manager_name = "csm_target";
    info.mode = ControlSignalInfo::MODE_TOPIC;
    info.type = ControlSignalInfo::TYPE_JOY;
    info.priority = 50;
    info.timeout_ns = 200 * kMs;
    info.disconnect_timeout_ns = 2000 * kMs;
    return info;
}

// V1: all defaults — valid.
TEST(InfoValidationTest, V1_DefaultsValid)
{
    const auto r = validateControlSignalInfo(makeR1Info());
    EXPECT_TRUE(r.valid);
    EXPECT_TRUE(r.error.empty());
}

// V2: empty controller_name — invalid, error names the field.
TEST(InfoValidationTest, V2_EmptyControllerName)
{
    auto info = makeR1Info();
    info.controller_name = "";
    const auto r = validateControlSignalInfo(info);
    EXPECT_FALSE(r.valid);
    EXPECT_NE(r.error.find("controller_name"), std::string::npos);
}

// V3: empty channel_name — invalid.
TEST(InfoValidationTest, V3_EmptyChannelName)
{
    auto info = makeR1Info();
    info.channel_name = "";
    const auto r = validateControlSignalInfo(info);
    EXPECT_FALSE(r.valid);
    EXPECT_NE(r.error.find("channel_name"), std::string::npos);
}

// V4: unknown mode — invalid.
TEST(InfoValidationTest, V4_UnknownMode)
{
    auto info = makeR1Info();
    info.mode = "unknown";
    const auto r = validateControlSignalInfo(info);
    EXPECT_FALSE(r.valid);
    EXPECT_NE(r.error.find("mode"), std::string::npos);
}

// V5: priority out of range — invalid (4 sub-cases).
TEST(InfoValidationTest, V5_PriorityOutOfRange)
{
    for (const int p : {0, -1, 101, 127})
    {
        auto info = makeR1Info();
        info.priority = static_cast<int8_t>(p);
        const auto r = validateControlSignalInfo(info);
        EXPECT_FALSE(r.valid) << "priority = " << p;
        EXPECT_NE(r.error.find("priority"), std::string::npos) << "priority = " << p;
    }
}

// V6: priority boundaries 1 and 100 — valid.
TEST(InfoValidationTest, V6_PriorityBoundaries)
{
    for (const int p : {1, 100})
    {
        auto info = makeR1Info();
        info.priority = static_cast<int8_t>(p);
        EXPECT_TRUE(validateControlSignalInfo(info).valid) << "priority = " << p;
    }
}

// V7: timeout_ns = 0 — valid in topic mode (disabled), invalid in service
// mode (response wait limit cannot be disabled). Both sides asserted.
TEST(InfoValidationTest, V7_TimeoutZeroTopicVsService)
{
    auto topic = makeR1Info();
    topic.timeout_ns = 0;
    topic.disconnect_timeout_ns = 2000 * kMs;  // variant C is fine
    EXPECT_TRUE(validateControlSignalInfo(topic).valid);

    auto service = makeR1Info();
    service.mode = ControlSignalInfo::MODE_SERVICE;
    service.timeout_ns = 0;
    const auto r = validateControlSignalInfo(service);
    EXPECT_FALSE(r.valid);
    EXPECT_NE(r.error.find("timeout_ns"), std::string::npos);
}

// V8: disconnect_timeout_ns == timeout_ns (both > 0) — invalid (strictly >).
TEST(InfoValidationTest, V8_DisconnectEqualsTimeout)
{
    auto info = makeR1Info();
    info.disconnect_timeout_ns = info.timeout_ns;
    const auto r = validateControlSignalInfo(info);
    EXPECT_FALSE(r.valid);
    EXPECT_NE(r.error.find("disconnect_timeout_ns"), std::string::npos);
}

// V9: disconnect_timeout_ns = 0 — valid (disabled, variant B).
TEST(InfoValidationTest, V9_DisconnectZeroValid)
{
    auto info = makeR1Info();
    info.disconnect_timeout_ns = 0;
    EXPECT_TRUE(validateControlSignalInfo(info).valid);
}

// V10: timeout_ns = 0 with disconnect_timeout_ns > 0 (topic) — valid
// (variant C).
TEST(InfoValidationTest, V10_VariantCValid)
{
    auto info = makeR1Info();
    info.timeout_ns = 0;
    info.disconnect_timeout_ns = 2000 * kMs;
    EXPECT_TRUE(validateControlSignalInfo(info).valid);
}

// V11: empty target_manager_name (registerSource path) — invalid.
TEST(InfoValidationTest, V11_EmptyTargetManager)
{
    auto info = makeR1Info();
    info.target_manager_name = "";
    const auto r = validateControlSignalInfo(info);
    EXPECT_FALSE(r.valid);
    EXPECT_NE(r.error.find("target_manager_name"), std::string::npos);
}

}  // namespace

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
