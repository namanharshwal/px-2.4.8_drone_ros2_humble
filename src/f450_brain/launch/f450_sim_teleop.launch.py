#!/usr/bin/env python3
# Copyright (c) 2025-2026 Naman Harshwal. All Rights Reserved.
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
"""
F450 Quadcopter Simulation & Teleoperation Bringup Launch File
Author: Naman Harshwal

Launches:
1. robot_state_publisher with complete 3D F450 URDF model
2. f450_sim hybrid 6-DOF simulation & hardware bridge engine
3. rviz2 3D digital twin visualization
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_dir = get_package_share_directory('f450_brain')
    urdf_path = os.path.join(pkg_dir, 'urdf', 'f450.urdf')
    rviz_config_path = os.path.join(pkg_dir, 'rviz', 'f450_sim.rviz')

    with open(urdf_path, 'r') as infp:
        robot_description_content = infp.read()

    return LaunchDescription([
        LogInfo(msg="=== Launching F450 Digital Twin Simulation & Hardware Bridge ==="),

        # 1. Robot State Publisher (3D Model URDF + TF Tree)
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description_content}]
        ),

        # 2. Hybrid Simulation & Hardware Synchronization Engine
        Node(
            package='f450_brain',
            executable='f450_sim',
            name='f450_sim_engine',
            output='screen'
        ),

        # 3. 3D Digital Twin Visualizer in RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_path]
        ),

        LogInfo(msg="=== RViz2 & Sim Engine Online! To control the drone, open a new terminal and run: ==="),
        LogInfo(msg="    ros2 run f450_brain keyboard_teleop")
    ])
