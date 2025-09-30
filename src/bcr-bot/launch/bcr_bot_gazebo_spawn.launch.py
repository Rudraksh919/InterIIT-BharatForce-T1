#!/usr/bin/env python3

from os.path import join
import xacro

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch.substitutions import PythonExpression

from launch_ros.actions import Node


def get_xacro_to_doc(xacro_file_path, mappings):
    doc = xacro.parse(open(xacro_file_path))
    xacro.process_doc(doc, mappings=mappings)
    return doc


def generate_launch_description():
    # Get bcr_bot package's share directory path
    bcr_bot_path = get_package_share_directory('bcr_bot')

    # Retrieve launch configuration arguments
    robot_positions = [
        {"x": "0.0", "y": "0.0", "yaw": "0.0"},
        {"x": "2.0", "y": "-3.2", "yaw": "1.57"},
        {"x": "-1.0", "y": "-3.7", "yaw": "1.75"},
    ]

    camera_enabled = LaunchConfiguration("camera_enabled", default=True)
    stereo_camera_enabled = LaunchConfiguration(
        "stereo_camera_enabled", default=False)
    two_d_lidar_enabled = LaunchConfiguration(
        "two_d_lidar_enabled", default=True)
    odometry_source = LaunchConfiguration("odometry_source", default="world")
    robot_namespace = LaunchConfiguration("robot_namespace", default='bcr_bot')

    nodes = []

    # Path to the Xacro file
    xacro_path = join(bcr_bot_path, 'urdf', 'multi_bcr_bot.xacro')
    # doc = get_xacro_to_doc(xacro_path, {"wheel_odom_topic": "odom", "sim_gazebo": "true", "two_d_lidar_enabled": "true", "camera_enabled": "true"})

    # # Launch the robot_state_publisher node
    # robot_state_publisher = Node(
    #     package='robot_state_publisher',
    #     executable='robot_state_publisher',
    #     name='robot_state_publisher',
    #     output='screen',
    #     parameters=[
    #         {'robot_description': Command(
    #             ['xacro ', xacro_path,
    #              ' camera_enabled:=', camera_enabled,
    #              ' stereo_camera_enabled:=', stereo_camera_enabled,
    #              ' two_d_lidar_enabled:=', two_d_lidar_enabled,
    #              ' sim_gazebo:=', "true",
    #              ' odometry_source:=', odometry_source,
    #              ' robot_namespace:=', robot_namespace,
    #              ])}],
    #     remappings=[
    #         ('/joint_states',
    #          PythonExpression(['"', robot_namespace, '/joint_states"'])),
    #     ]
    # )

    # # Launch the spawn_entity node to spawn the robot in Gazebo
    # spawn_entity = Node(
    #     package='gazebo_ros',
    #     executable='spawn_entity.py',
    #     output='screen',
    #     arguments=[
    #         '-topic', "/robot_description",
    #         # default enitity name _bcr_bot
    #         '-entity', PythonExpression(['"', robot_namespace, '_robot"']),
    #         '-z', "0.28",
    #         '-x', position_x,
    #         '-y', position_y,
    #         '-Y', orientation_yaw
    #     ]
    # )

    for i, pos in enumerate(robot_positions):
        namespace = f"bcr_bot_{i+1}"

        # robot_state_publisher
        rsp_node = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace=namespace,
            output='screen',
            parameters=[{
                'robot_description': Command([
                    'xacro ', xacro_path,
                    ' camera_enabled:=', camera_enabled,
                    ' stereo_camera_enabled:=', stereo_camera_enabled,
                    ' two_d_lidar_enabled:=', two_d_lidar_enabled,
                    ' sim_gazebo:=', "true",
                    ' odometry_source:=', odometry_source,
                    ' robot_namespace:=', namespace,
                ])
            }],
            remappings=[
                ('/joint_states',
                 PythonExpression(['"', namespace, '/joint_states"'])),
            ]
        )

        # spawn_entity node
        spawn_node = Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            output='screen',
            arguments=[
                '-topic', f"/{namespace}/robot_description",
                '-entity', f"{namespace}_robot",
                '-x', pos["x"],
                '-y', pos["y"],
                '-Y', pos["yaw"]
            ]
        )

        nodes.extend([rsp_node, spawn_node])

    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument("camera_enabled", default_value=camera_enabled),
        DeclareLaunchArgument("stereo_camera_enabled",
                              default_value=stereo_camera_enabled),
        DeclareLaunchArgument("two_d_lidar_enabled",
                              default_value=two_d_lidar_enabled),
        # DeclareLaunchArgument("position_x", default_value="0.0"),
        # DeclareLaunchArgument("position_y", default_value="0.0"),
        # DeclareLaunchArgument("orientation_yaw", default_value="0.0"),
        DeclareLaunchArgument(
            "odometry_source", default_value=odometry_source),
        # DeclareLaunchArgument(
        #     "robot_namespace", default_value=robot_namespace),
        # DeclareLaunchArgument('robot_description', default_value=doc.toxml()),
        # robot_state_publisher,
        # spawn_entity
        *nodes

    ])
