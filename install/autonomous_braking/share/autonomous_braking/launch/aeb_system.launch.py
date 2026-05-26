#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription([
        LogInfo(msg='Starting Autonomous Emergency Braking System...'),
        LogInfo(msg='Press Q in the visualizer window to exit'),

        Node(
            package='autonomous_braking',
            executable='simulation',
            name='world_simulation',
            output='screen',
        ),
        Node(
            package='autonomous_braking',
            executable='visualizer',
            name='aeb_visualizer',
            output='screen',
        ),
    ])
