#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        LogInfo(msg=["Execute tribot_spawn.launch.py!"]),
        Node(
            package='tribot_description',
            executable='inject_remap_entity.py',
            output='screen',
            arguments=[
                '--robot-urdf', LaunchConfiguration('robot-urdf'),
                '--robot-name', LaunchConfiguration('robot-name'),
                '--robot-namespace', LaunchConfiguration('robot-namespace'),
                '-x', LaunchConfiguration('x'),
                '-y', LaunchConfiguration('y'),
                '-z', LaunchConfiguration('z'),]
        ),
    ])