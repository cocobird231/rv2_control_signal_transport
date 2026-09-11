// ============================================================
//  control_signal_types.cpp
//
//  The single translation unit that binds concrete rv2 control-signal
//  message/service types to the ControlSignalFactory registry.
//
//  To add a new control-signal type
//  ───────────────────────────────────
//  1. Include the ROS 2 message header (and service header if needed):
//
//       #include <my_msgs/msg/my_type.hpp>
//       #include <rv2_interfaces/srv/control_signal_my_type.hpp>  // optional
//
//  2. Call REGISTER_CONTROL_SIGNAL with four arguments (at file scope,
//     outside any namespace):
//
//       REGISTER_CONTROL_SIGNAL(
//           MyType,                                        // UniqueId  — bare identifier
//           "my_type",                                     // name_str  — runtime string
//           my_msgs::msg::MyType,                          // MsgType
//           rv2_interfaces::srv::ControlSignalMyType);     // SrvType (or void)
//
//  No other changes are required anywhere in the codebase.
//
//  Build this file into a SHARED library (see CMakeLists.txt).
// ============================================================

#include "rv2_control_signal_transport/control_signal_factory.h"

// Concrete message types.
#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <std_msgs/msg/string.hpp>

// Concrete service types (service-mode transport).
#include <rv2_interfaces/srv/control_signal_joy.hpp>
#include <rv2_interfaces/srv/control_signal_twist.hpp>

// ── Registrations ──────────────────────────────────────────────────────────────
// Each macro call inserts a static object that self-registers at program startup.
// The anonymous namespace inside the macro is intentional — it confines the
// auto-register struct to this translation unit.

REGISTER_CONTROL_SIGNAL(Joy, "joy", sensor_msgs::msg::Joy, rv2_interfaces::srv::ControlSignalJoy);

REGISTER_CONTROL_SIGNAL(Twist, "twist", geometry_msgs::msg::Twist, rv2_interfaces::srv::ControlSignalTwist);

// String is topic-only — pass void as SrvType.
REGISTER_CONTROL_SIGNAL(String, "string", std_msgs::msg::String, void);
