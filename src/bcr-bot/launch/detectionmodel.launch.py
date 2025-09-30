#!/usr/bin/env python3

from os.path import join
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import SetEnvironmentVariable
from launch.actions import AppendEnvironmentVariable
from launch_ros.actions import Node


def generate_launch_description():
    bcr_bot_path = get_package_share_directory('bcr_bot')

    return LaunchDescription([
        Node(
            package='bcr_bot',       # package name
            executable='objdetection.py',  # your Python node
            name='cone_detector_node',  # node name
            output='screen',          # output to screen
        ),
    ])
