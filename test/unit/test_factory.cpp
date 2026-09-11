/**
 * @file test_factory.cpp
 * @brief F1-F5 unit tests for r1::ControlSignalFactory (design §7.3).
 *
 * Single node; rclcpp init required for entity creation, no cross-node
 * traffic. Built-in registrations (joy/twist/string) come from the shared
 * library (control_signal_types.cpp), which F5 verifies across TUs.
 */

#include <string>
#include <typeindex>

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <std_msgs/msg/string.hpp>

#include "r1_test_utils.h"
#include "rv2_control_signal_transport/r1/control_signal_factory.h"

namespace
{

using rv2_interfaces::r1::ControlSignalFactory;
using rv2_interfaces::r1::ControlSignalInfo;
using rv2_interfaces::r1::makeTransportInfo;

constexpr int64_t kMs = 1'000'000;

class FactoryTest : public rv2_interfaces::r1::CsmTestBase
{
protected:
    ControlSignalInfo info(const std::string& channel, const std::string& mode, const std::string& type)
    {
        return makeTransportInfo(channel, mode, type, 200 * kMs, 2000 * kMs);
    }
};

// F1: Create registered types, topic and service mode — non-null and
// msgType() reports the right type.
TEST_F(FactoryTest, F1_CreateRegisteredTypes)
{
    auto& f = ControlSignalFactory::Instance();
    std::string err;

    auto srcTopic = f.CreateSource("joy", node_.get(), info("f1/a", ControlSignalInfo::MODE_TOPIC, "joy"), &err);
    ASSERT_NE(srcTopic, nullptr) << err;
    EXPECT_EQ(srcTopic->msgType(), std::type_index(typeid(sensor_msgs::msg::Joy)));

    auto srcService = f.CreateSource("joy", node_.get(), info("f1/b", ControlSignalInfo::MODE_SERVICE, "joy"), &err);
    ASSERT_NE(srcService, nullptr) << err;

    auto sinkTopic = f.CreateSink("twist", node_.get(), info("f1/c", ControlSignalInfo::MODE_TOPIC, "twist"), &err);
    ASSERT_NE(sinkTopic, nullptr) << err;
    EXPECT_EQ(sinkTopic->msgType(), std::type_index(typeid(geometry_msgs::msg::Twist)));

    auto sinkService = f.CreateSink("twist", node_.get(), info("f1/d", ControlSignalInfo::MODE_SERVICE, "twist"), &err);
    ASSERT_NE(sinkService, nullptr) << err;
}

// F2: Create an unregistered type — nullptr plus an error string, and no
// exception escapes.
TEST_F(FactoryTest, F2_UnregisteredTypeNoThrow)
{
    auto& f = ControlSignalFactory::Instance();
    std::string err;
    std::shared_ptr<rv2_interfaces::r1::BaseControlSignalSource> src;
    EXPECT_NO_THROW(src =
                        f.CreateSource("nope", node_.get(), info("f2/a", ControlSignalInfo::MODE_TOPIC, "nope"), &err));
    EXPECT_EQ(src, nullptr);
    EXPECT_FALSE(err.empty());

    // Topic-only "string" refused in service mode, same non-throwing contract.
    err.clear();
    auto sink = f.CreateSink("string", node_.get(), info("f2/b", ControlSignalInfo::MODE_SERVICE, "string"), &err);
    EXPECT_EQ(sink, nullptr);
    EXPECT_FALSE(err.empty());
}

// F3: typeKey reverse lookup — joy/twist/string and unregistered type.
TEST_F(FactoryTest, F3_TypeKeyReverseLookup)
{
    auto& f = ControlSignalFactory::Instance();
    EXPECT_EQ(f.typeKey(std::type_index(typeid(sensor_msgs::msg::Joy))), "joy");
    EXPECT_EQ(f.typeKey(std::type_index(typeid(geometry_msgs::msg::Twist))), "twist");
    EXPECT_EQ(f.typeKey(std::type_index(typeid(std_msgs::msg::String))), "string");
    EXPECT_EQ(f.typeKey(std::type_index(typeid(int))), "");
}

// F4: duplicate Register — false, original entry intact.
TEST_F(FactoryTest, F4_DuplicateRegisterRefused)
{
    auto& f = ControlSignalFactory::Instance();
    // "joy" is already registered by the shared library.
    EXPECT_FALSE((f.Register<geometry_msgs::msg::Twist, void>("joy")));
    // The original mapping survives: msgType is still Joy.
    std::string err;
    auto src = f.CreateSource("joy", node_.get(), info("f4/a", ControlSignalInfo::MODE_TOPIC, "joy"), &err);
    ASSERT_NE(src, nullptr) << err;
    EXPECT_EQ(src->msgType(), std::type_index(typeid(sensor_msgs::msg::Joy)));
}

// F5: singleton consistency across TUs — registrations done inside the
// shared library (control_signal_types.cpp) are visible to this test TU.
TEST_F(FactoryTest, F5_SingletonAcrossTUs)
{
    auto& f = ControlSignalFactory::Instance();
    EXPECT_TRUE(f.has("joy"));
    EXPECT_TRUE(f.has("twist"));
    EXPECT_TRUE(f.has("string"));
    EXPECT_FALSE(f.has("nope"));
    // Same singleton object on repeated access.
    EXPECT_EQ(&f, &ControlSignalFactory::Instance());
}

}  // namespace

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    ::testing::AddGlobalTestEnvironment(new rv2_interfaces::r1::RclcppEnv);
    return RUN_ALL_TESTS();
}
