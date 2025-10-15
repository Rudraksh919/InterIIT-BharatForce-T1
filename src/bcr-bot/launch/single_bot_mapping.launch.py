# single_bot_slam.launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # set robot namespace here (or make it a LaunchConfiguration)
    robot_ns = '/bcr_bot_1'   # change to /bcr_bot_2 when mapping robot2

    params = {
        'use_sim_time': False,
        # you can add slam params here or load a yaml file via parameters=[...]
    }

    return LaunchDescription([
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',   # use async node for online mapping
            name='slam_toolbox',
            output='screen',
            parameters=[params],
            remappings=[
                ('scan', f'{robot_ns}/scan'),
                ('odom', f'{robot_ns}/odom'),
                # optionally remap 'tf' topics only if you have namespaced tf publishers
            ],
        )
    ])
