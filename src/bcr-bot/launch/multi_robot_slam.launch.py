# import os
# from ament_index_python.packages import get_package_share_directory
# from launch import LaunchDescription
# from launch_ros.actions import Node

# def generate_launch_description():
#     # Define the path to your configuration file
#     # Replace 'your_package_name' with the name of your ROS 2 package
#     params_file = os.path.join(
#         get_package_share_directory('bcr_bot'),
#         'config',
#         'multi_mapper.yaml'
#     )

#     # Node for running the asynchronous SLAM toolbox
#     start_async_slam_toolbox_node = Node(
#         package='slam_toolbox',
#         executable='async_slam_toolbox_node', # <-- Use the async node!
#         name='slam_toolbox',
#         output='screen',
#         parameters=[params_file],
#         remappings=[
#           ('/map', '/map') # Optional: ensure map topic is not namespaced
#         ]
#     )

#     return LaunchDescription([
#         start_async_slam_toolbox_node
#     ])

# multi_robot_slam_launch.py
from launch import LaunchDescription
from launch_ros.actions import Node

import os
from ament_index_python.packages import get_package_share_directory

# ... inside your generate_launch_description() function ...

# Define the name of your package
package_name = 'bcr_bot'

# Construct the full path to your YAML file
params_file = os.path.join(
    get_package_share_directory(package_name),
    'config',
    'robot_1_slam.yaml'  # <-- Replace with your actual .yaml file name
)


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            namespace='robot_1',
            # remappings=[('/map','map')],
            parameters=[params_file],
        ),
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            namespace='robot_2',
            #remappings=[('/map','map')],
            parameters=[params_file],
        ),
        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            namespace='robot_3',
            #remappings=[('/map','map')],
            parameters=[params_file],
        ),
    ])

