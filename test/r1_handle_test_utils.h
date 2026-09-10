/**
 * @file r1_handle_test_utils.h
 * @brief Shared manager/probe fixture for Handle contracts and retry lifecycle tests.
 */
#pragma once

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

class HandleTestBase : public ::testing::Test
{
protected:
    explicit HandleTestBase(const char* prefix) : prefix_(prefix) {}

    void SetUp() override
    {
        uid_ = prefix_ + std::to_string(counter_++);
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

    const std::string prefix_;
    static int counter_;
    std::string uid_;
    rclcpp::Node::SharedPtr nodeA_, nodeB_, auxNode_;
    std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
    std::thread spin_;
    rclcpp::Service<CsmRegisterSrv>::SharedPtr masterReg_;
    rclcpp::Service<CsmHeartbeatSrv>::SharedPtr masterHb_;
    std::unique_ptr<ControlSignalManager> mgrA_, mgrB_;
};
int HandleTestBase::counter_ = 0;

} // namespace
