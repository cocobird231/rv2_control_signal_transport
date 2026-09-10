/**
 * @file r1_test_utils.h
 * @brief Shared r1 test scaffolding (design §5.4): CsmTestBase with a
 *        background MultiThreadedExecutor, and the test-only
 *        ManagerTestAccess friend channel that simulates one CSM tick
 *        (_calcStatus + seal + _applyStatus) without a real Manager.
 */

#ifndef RV2_CONTROL_SIGNAL_TRANSPORT_TEST_R1_TEST_UTILS_H
#define RV2_CONTROL_SIGNAL_TRANSPORT_TEST_R1_TEST_UTILS_H

#include <memory>
#include <string>
#include <thread>

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include "rv2_control_signal_transport/r1/control_signal_info.h"
#include "rv2_control_signal_transport/r1/control_signal_sink.h"
#include "rv2_control_signal_transport/r1/control_signal_source.h"

namespace rv2_interfaces
{
namespace r1
{

/// Test-only friend channel (§5.4): compiled into test targets exclusively.
/// Grants construction (the constructors are private) and drives the D8 tick
/// sequence exactly the way a real CSM status tick would.
struct ManagerTestAccess
{
    template<typename msgT, typename srvT = void>
    static std::shared_ptr<ControlSignalSource<msgT, srvT>> createSource(
        rclcpp::Node* node, const ControlSignalInfo& info,
        int64_t rateWindowNs = 1'000'000'000)
    {
        return std::shared_ptr<ControlSignalSource<msgT, srvT>>(
            new ControlSignalSource<msgT, srvT>(node, info, rateWindowNs));
    }

    template<typename msgT, typename srvT = void>
    static std::shared_ptr<ControlSignalSink<msgT, srvT>> createSink(
        rclcpp::Node* node, const ControlSignalInfo& info,
        int64_t rateWindowNs = 1'000'000'000)
    {
        auto sink = std::shared_ptr<ControlSignalSink<msgT, srvT>>(
            new ControlSignalSink<msgT, srvT>(node, info, rateWindowNs));
        sink->_bindTransport();
        return sink;
    }

    template<typename Entity>
    static EntityDecision calc(const Entity& e, int64_t nowNs)
    {
        return e._calcStatus(nowNs);
    }

    template<typename Entity>
    static void apply(Entity& e, const EntityDecision& d)
    {
        e._applyStatus(d);
    }

    template<typename Entity>
    static bool trySealLocalTerminal(Entity& e, const EntityDecision& d)
    {
        return e._trySealLocalTerminal(d);
    }

    template<typename Entity>
    static void sealTerminal(Entity& e)
    {
        e._sealTerminal();
    }

    /// S14/S16: inject service outcomes directly so out-of-order completion
    /// interleavings are deterministic (the network cannot be ordered).
    template<typename SourceT>
    static void recordOutcomeFailure(SourceT& s, uint64_t seq, int64_t nowNs)
    {
        s._recordOutcomeFailure(seq, nowNs);
    }

    template<typename SourceT>
    static void recordOutcomeSuccess(SourceT& s, uint64_t seq)
    {
        s._recordOutcomeSuccess(seq);
    }

    /// One CSM tick equivalent (§8.3 phases 1-3 for a single entity):
    /// calc → (terminal ? trySeal, cancel on failure) → apply.
    /// Returns the decision actually applied (nullopt = terminal cancelled).
    template<typename Entity>
    static std::optional<EntityDecision> tick(Entity& e, int64_t nowNs)
    {
        EntityDecision d = e._calcStatus(nowNs);
        if (d.status.state == ControlSignalState::DISCONNECTED)
        {
            if (!e._trySealLocalTerminal(d))
                return std::nullopt;   // evidence changed: cancel this round
        }
        e._applyStatus(d);
        return d;
    }
};

/// gtest base: one node + MultiThreadedExecutor spinning in the background
/// (r1 flavour of the legacy CsmTestBase, §5.4).
class CsmTestBase : public ::testing::Test
{
protected:
    void SetUp() override
    {
        node_ = std::make_shared<rclcpp::Node>(
            "r1_test_" + std::to_string(reinterpret_cast<uintptr_t>(this) % 100000));
        executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>(
            rclcpp::ExecutorOptions(), 2);
        executor_->add_node(node_);
        spinThread_ = std::thread([this] { executor_->spin(); });
        // cancel() issued before spin() has started is lost and the join in
        // TearDown hangs forever — wait until the executor really spins.
        while (!executor_->is_spinning())
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    void TearDown() override
    {
        executor_->cancel();
        if (spinThread_.joinable())
            spinThread_.join();
        executor_.reset();
        node_.reset();
    }

    rclcpp::Node::SharedPtr node_;
    std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
    std::thread spinThread_;
};

/// Valid r1 Info defaults for transport tests; thresholds deliberately short
/// so timeout cases need no long sleeps (ticks inject fake now instead).
inline ControlSignalInfo makeTransportInfo(const std::string& channel,
                                           const std::string& mode,
                                           const std::string& type,
                                           int64_t timeoutNs,
                                           int64_t disconnectNs)
{
    ControlSignalInfo info;
    info.controller_name       = "ctrl_" + channel;
    info.channel_name          = channel;
    info.target_manager_name   = "csm_test_target";
    info.mode                  = mode;
    info.type                  = type;
    info.priority              = 50;
    info.timeout_ns            = timeoutNs;
    info.disconnect_timeout_ns = disconnectNs;
    return info;
}

/// Global environment: init/shutdown rclcpp once per test binary.
class RclcppEnv : public ::testing::Environment
{
public:
    void SetUp() override { rclcpp::init(0, nullptr); }
    void TearDown() override { rclcpp::shutdown(); }
};

} // namespace r1
} // namespace rv2_interfaces

#endif // RV2_CONTROL_SIGNAL_TRANSPORT_TEST_R1_TEST_UTILS_H
