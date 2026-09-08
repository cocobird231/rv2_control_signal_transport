/**
 * @file control_signal_types.cpp
 * @brief Built-in r1 control-signal type registrations (design draft §7.2):
 *        joy, twist and string. string is topic-only — its service type
 *        parameter is void.
 */

#include <std_msgs/msg/string.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joy.hpp>

#include <r1_interfaces/srv/control_signal_joy.hpp>
#include <r1_interfaces/srv/control_signal_twist.hpp>

#include "rv2_control_signal_transport/r1/control_signal_factory.h"

R1_REGISTER_CONTROL_SIGNAL(r1_joy, "joy", sensor_msgs::msg::Joy,
                           r1_interfaces::srv::ControlSignalJoy)
R1_REGISTER_CONTROL_SIGNAL(r1_twist, "twist", geometry_msgs::msg::Twist,
                           r1_interfaces::srv::ControlSignalTwist)
R1_REGISTER_CONTROL_SIGNAL(r1_string, "string", std_msgs::msg::String, void)
