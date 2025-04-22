#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    return LaunchDescription([
        # 1) ouster_ros driver
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                FindPackageShare('ouster_ros'),
                '/launch/driver.launch.py'
            ])
        ),
        # 2) scout_base’s omni base
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                FindPackageShare('scout_base'),
                '/launch/scout_mini_omni_base.launch.py'
            ])
        ),
        # 3) scout_description’s base description
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                FindPackageShare('scout_description'),
                '/launch/scout_base_description.launch.py'
            ])
        ),
    ])
