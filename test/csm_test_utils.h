// ============================================================
//  csm_test_utils.h
//
//  Shared gtest helpers for the rv2_control_signal_transport unit tests:
//    - makeInfo()  : build a ControlSignalInfo with sensible defaults.
//    - stateName() : human-readable ControlSignalState.
//    - RclcppEnvironment : inits rclcpp once per binary.
//    - CsmTestBase : fixture with a MultiThreadedExecutor spinning in the
//                    background and node-creation helpers.
// ============================================================

#ifndef RV2_CONTROL_SIGNAL_TRANSPORT__TEST__CSM_TEST_UTILS_H_
#define RV2_CONTROL_SIGNAL_TRANSPORT__TEST__CSM_TEST_UTILS_H_

#include <atomic>
#include <chrono>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include "rv2_control_signal_transport/control_signal_manager.h"

namespace rv2_test
{

using namespace std::chrono_literals;

/// @brief Human-readable name for a ControlSignalState (handy in EXPECT messages).
inline const char* stateName(rv2_interfaces::ControlSignalState s)
{
    using S = rv2_interfaces::ControlSignalState;
    switch (s)
    {
        case S::UNKNOWN:      return "UNKNOWN";
        case S::ACTIVE:       return "ACTIVE";
        case S::LOW_FREQ:     return "LOW_FREQ";
        case S::TIMEOUT:      return "TIMEOUT";
        case S::DISCONNECTED: return "DISCONNECTED";
    }
    return "?";
}

/// @brief Build a ControlSignalInfo with the same defaults the legacy test used.
inline rv2_interfaces::msg::ControlSignalInfo makeInfo(
    const std::string& channel,
    const std::string& type,
    const std::string& mode,
    const std::string& targetCsm           = "",
    bool               useKeepAlive         = false,
    int64_t            keepAliveNs          = 300'000'000LL,    // 300 ms
    int64_t            timeoutNs            = 2'000'000'000LL,  // 2 s
    float              sendFreqHz           = 0.0f,            // 0 = disabled
    int64_t            disconnectTimeoutNs  = 0)               // 0 = no auto-disconnect
{
    rv2_interfaces::msg::ControlSignalInfo info;
    info.channel_name           = channel;
    info.control_signal_type    = type;
    info.control_signal_mode    = mode;
    info.target_csm_name        = targetCsm;
    info.use_keep_alive         = useKeepAlive;
    info.keep_alive_interval_ns = keepAliveNs;
    info.send_freq_hz           = sendFreqHz;
    info.timeout_ns             = timeoutNs;
    info.disconnect_timeout_ns  = disconnectTimeoutNs;
    return info;
}

/**
 * @brief Global test environment: initialises rclcpp once for the whole test
 *        binary.
 *
 * Registered via ::testing::AddGlobalTestEnvironment() in each test main.
 */
class RclcppEnvironment : public ::testing::Environment
{
public:
    void SetUp() override
    {
        if (!rclcpp::ok())
            rclcpp::init(0, nullptr);
    }

    void TearDown() override
    {
        if (rclcpp::ok())
            rclcpp::shutdown();
    }
};

/**
 * @brief Test fixture that runs a MultiThreadedExecutor in a background thread.
 *
 * Nodes created via makeNode() are added to the executor and kept alive for the
 * duration of the test; they are removed and destroyed in TearDown(). The
 * background executor lets ROS 2 topic/service callbacks dispatch while the test
 * thread blocks on registerSource() or sleeps waiting for state transitions.
 */
class CsmTestBase : public ::testing::Test
{
protected:
    void SetUp() override
    {
        exec_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>();
        spinning_ = true;
        spinThread_ = std::thread([this]() { exec_->spin(); });

        // Wait until the executor has actually entered its spin loop before the
        // test body runs. Without this, a test that does no ROS I/O (e.g. a pure
        // factory lookup) can reach TearDown and call cancel() *before* spin()
        // begins waiting — the cancel is then lost and spinThread_.join() blocks
        // forever (cancel-before-spin race). Bounded so a stuck executor cannot
        // hang the suite indefinitely.
        const auto deadline = std::chrono::steady_clock::now() + 5s;
        while (!exec_->is_spinning() &&
               std::chrono::steady_clock::now() < deadline)
        {
            std::this_thread::sleep_for(1ms);
        }
    }

    void TearDown() override
    {
        if (spinning_)
        {
            // cancel() is safe now that SetUp() guaranteed the executor is
            // spinning; the join() will return promptly.
            exec_->cancel();
            if (spinThread_.joinable())
                spinThread_.join();
            spinning_ = false;
        }
        for (auto& n : nodes_)
            exec_->remove_node(n);
        nodes_.clear();
        exec_.reset();
    }

    /// @brief Create a node, add it to the running executor, and retain it.
    rclcpp::Node::SharedPtr makeNode(const std::string& name)
    {
        auto node = rclcpp::Node::make_shared(name);
        exec_->add_node(node);
        nodes_.push_back(node);
        return node;
    }

    std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> exec_;
    std::vector<rclcpp::Node::SharedPtr>                      nodes_;
    std::thread                                              spinThread_;
    bool                                                    spinning_ = false;
};

}  // namespace rv2_test

#endif  // RV2_CONTROL_SIGNAL_TRANSPORT__TEST__CSM_TEST_UTILS_H_
