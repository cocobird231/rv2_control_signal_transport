/**
 * @file csm_master.cpp
 * @brief csm_master_node — standalone executable hosting r1::CsmMaster
 *        (design §9, §2.1). Parameters map onto MasterOptions.
 */

#include <memory>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "rv2_control_signal_transport/r1/csm_master.h"

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("csm_master_node");

    node->declare_parameter<std::string>("master_name", "csm_master");
    node->declare_parameter<int64_t>("tick_interval_ms", 200);
    node->declare_parameter<int64_t>("pair_grace_ms", 1000);
    node->declare_parameter<int64_t>("notify_initial_delay_ms", 200);
    node->declare_parameter<int64_t>("notify_max_delay_ms", 5000);
    node->declare_parameter<double>("notify_jitter_ratio", 0.2);
    node->declare_parameter<int64_t>("notify_max_in_flight", 8);
    node->declare_parameter("csm_whitelist", std::vector<std::string>{});
    node->declare_parameter("csm_blacklist", std::vector<std::string>{});

    rv2_interfaces::r1::NotificationRetryPolicy retry(
        node->get_parameter("notify_initial_delay_ms").as_int(),
        node->get_parameter("notify_max_delay_ms").as_int(),
        node->get_parameter("notify_jitter_ratio").as_double(),
        static_cast<uint32_t>(node->get_parameter("notify_max_in_flight").as_int()));
    rv2_interfaces::r1::MasterOptions opt(retry);
    opt.masterName = node->get_parameter("master_name").as_string();
    opt.tickIntervalMs = node->get_parameter("tick_interval_ms").as_int();
    opt.pairGraceMs = node->get_parameter("pair_grace_ms").as_int();

    rv2_interfaces::r1::CsmMaster master(node.get(), opt);
    const auto whitelist =
        node->get_parameter("csm_whitelist").as_string_array();
    if (!whitelist.empty())
        master.enableCsmWhitelist(whitelist);
    const auto blacklist =
        node->get_parameter("csm_blacklist").as_string_array();
    if (!blacklist.empty())
        master.enableCsmBlacklist(blacklist);

    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
