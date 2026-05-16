"""
keyboard_source_twist.launch.py

Thin wrapper that launches KeyboardSourceNode in Twist mode by delegating
to keyboard_source.launch.py with initial_msg_type fixed to "twist".

All other arguments (server_name, priority, …) are declared in the base
launch file and can still be overridden on the command line.

For Joy mode use: keyboard_source_joy.launch.py
Full argument reference: keyboard_source.launch.py

Usage
─────
  ros2 launch rv2_control_signal_transport keyboard_source_twist.launch.py
  ros2 launch rv2_control_signal_transport keyboard_source_twist.launch.py \\
      server_name:=my_robot  priority:=80
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('rv2_control_signal_transport'),
                    'launch', 'keyboard_source.launch.py',
                ])
            ),
            launch_arguments={
                'config_file': PathJoinSubstitution([
                    FindPackageShare('rv2_control_signal_transport'),
                    'config',
                    'keyboard_source_twist.yaml',
                ]),
            }.items(),
        ),
    ])
