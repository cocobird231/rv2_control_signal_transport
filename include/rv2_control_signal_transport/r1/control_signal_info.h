/**
 * @file control_signal_info.h
 * @brief r1 Info alias and validateControlSignalInfo() — the six generic
 *        validation rules of design draft §3.2.
 *
 * Besides the generated message header there is no ROS dependency (§3.1);
 * the validation is pure logic and unit-testable without rclcpp init.
 */

#ifndef RV2_CONTROL_SIGNAL_TRANSPORT_R1_CONTROL_SIGNAL_INFO_H
#define RV2_CONTROL_SIGNAL_TRANSPORT_R1_CONTROL_SIGNAL_INFO_H

#include <string>

#include <r1_interfaces/msg/control_signal_info.hpp>

namespace rv2_interfaces
{
namespace r1
{

/// Message alias (§3.1). Interfaces live in the standalone r1_interfaces
/// package (§12 #1 decision); this alias keeps user code package-agnostic.
using ControlSignalInfo = r1_interfaces::msg::ControlSignalInfo;
using InfoT = ControlSignalInfo;

/// Mode / type string constants re-exported from the message definition
/// (rosidl generates them as static const std::string).
inline const std::string& kModeTopic   = ControlSignalInfo::MODE_TOPIC;
inline const std::string& kModeService = ControlSignalInfo::MODE_SERVICE;
inline const std::string& kTypeJoy     = ControlSignalInfo::TYPE_JOY;
inline const std::string& kTypeTwist   = ControlSignalInfo::TYPE_TWIST;
inline const std::string& kTypeString  = ControlSignalInfo::TYPE_STRING;

/// Result of validateControlSignalInfo(); error names the offending field.
struct InfoValidation
{
    bool valid;
    std::string error;
};

/**
 * @brief The six generic validation rules (§3.2). ManagerOptions' CSM-level
 *        thresholds and retry policy validate themselves elsewhere (§8.2)
 *        and are deliberately not mixed in here.
 *
 * 1. controller_name must not be empty.
 * 2. channel_name must not be empty.
 * 3. mode must be "topic" or "service", and type must not be empty.
 * 4. priority must lie in [1, 100]; 0 is invalid, negatives and >100 rejected.
 * 5. timeout_ns >= 0 and disconnect_timeout_ns >= 0 (0 disables each layer,
 *    §2.3.1). When both are > 0, disconnect_timeout_ns > timeout_ns strictly.
 *    In service mode timeout_ns > 0 (the response wait limit cannot be
 *    disabled).
 * 6. target_manager_name must not be empty on the registerSource path
 *    (_onManage(REGISTER) additionally checks it equals the local Manager
 *    name as mis-route protection — that side is not part of this function).
 */
inline InfoValidation validateControlSignalInfo(const ControlSignalInfo& info)
{
    // Rule 1
    if (info.controller_name.empty())
        return {false, "controller_name must not be empty"};

    // Rule 2
    if (info.channel_name.empty())
        return {false, "channel_name must not be empty"};

    // Rule 3
    const bool isTopic = info.mode == ControlSignalInfo::MODE_TOPIC;
    const bool isService = info.mode == ControlSignalInfo::MODE_SERVICE;
    if (!isTopic && !isService)
        return {false, "mode must be \"topic\" or \"service\" (got \"" +
                           info.mode + "\")"};
    if (info.type.empty())
        return {false, "type must not be empty"};

    // Rule 4
    if (info.priority < 1 || info.priority > 100)
        return {false, "priority (" + std::to_string(static_cast<int>(info.priority)) +
                           ") must be within [1, 100]; 0 = invalid"};

    // Rule 5
    if (info.timeout_ns < 0)
        return {false, "timeout_ns must be >= 0 (0 = disabled)"};
    if (info.disconnect_timeout_ns < 0)
        return {false, "disconnect_timeout_ns must be >= 0 (0 = disabled)"};
    if (info.timeout_ns > 0 && info.disconnect_timeout_ns > 0 &&
        info.disconnect_timeout_ns <= info.timeout_ns)
        return {false, "disconnect_timeout_ns (" +
                           std::to_string(info.disconnect_timeout_ns) +
                           ") must be strictly greater than timeout_ns (" +
                           std::to_string(info.timeout_ns) + ")"};
    if (isService && info.timeout_ns == 0)
        return {false, "timeout_ns must be > 0 in service mode "
                       "(response wait limit cannot be disabled)"};

    // Rule 6
    if (info.target_manager_name.empty())
        return {false, "target_manager_name must not be empty on the "
                       "registerSource path"};

    return {true, ""};
}

} // namespace r1
} // namespace rv2_interfaces

#endif // RV2_CONTROL_SIGNAL_TRANSPORT_R1_CONTROL_SIGNAL_INFO_H
