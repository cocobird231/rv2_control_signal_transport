"""
keyboard_source.launch.py

Launches KeyboardSourceNode in its own multi-threaded container so that
ControlSignalManager::registerSource() (which blocks) can call back into
the ROS 2 executor without deadlocking.

The node loads all ROS 2 parameters from a YAML file selected by `config_file`.

Usage
─────
  ros2 launch rv2_control_signal_transport keyboard_source.launch.py

  ros2 launch rv2_control_signal_transport keyboard_source.launch.py \\
      config_file:=/path/to/keyboard_source.yaml

Launch arguments
────────────────
    config_file       Path to ROS 2 parameter YAML (default: config/keyboard_source.yaml)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # ── Launch argument declarations ──────────────────────────────────────────
    args = [
        DeclareLaunchArgument(
            'config_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('rv2_control_signal_transport'),
                'config',
                'keyboard_source.yaml',
            ]),
            description='Path to KeyboardSourceNode ROS 2 parameter YAML file'),
    ]

    # ── Composable node (multi-threaded container required) ───────────────────
    container = ComposableNodeContainer(
        name='keyboard_source_container',
        namespace='',
        package='rclcpp_components',
        # component_container_mt uses MultiThreadedExecutor, which is required
        # because KeyboardSourceNode::_init() calls registerSource() — a blocking
        # service call — from a timer callback that runs on the same executor.
        executable='component_container_mt',
        composable_node_descriptions=[
            ComposableNode(
                package='rv2_control_signal_transport',
                plugin='KeyboardSourceNode',
                name='keyboard_source',
                parameters=[
                    LaunchConfiguration('config_file'),
                ],
            ),
        ],
        output='screen',
    )

    return LaunchDescription(args + [container])
