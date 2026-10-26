#!/usr/bin/env python3
"""
F450 Drone Bringup Launch File
Launches the Drone Visualizer and optionally the Drone Commander.
"""

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='f450_brain',
            executable='visualizer',
            name='drone_visualizer',
            output='screen'
        ),
    ])
