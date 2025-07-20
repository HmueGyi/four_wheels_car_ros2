#!/usr/bin/env python3

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Package directories
    gazebo_ros_pkg = get_package_share_directory('gazebo_ros')
    robot_desc_pkg = get_package_share_directory('four_wheels_robot_description')
    robot_gazebo_pkg = get_package_share_directory('four_wheels_robot_gazebo')

    # Paths
    empty_world = os.path.join(gazebo_ros_pkg, 'worlds', 'empty.world')
    controller_config = os.path.join(robot_desc_pkg, 'config', 'my_controllers.yaml')
    twist_mux_config = os.path.join(robot_gazebo_pkg, 'config', 'twist_mux.yaml')

    # Launch args for robot spawn position
    declare_x_pose = DeclareLaunchArgument('x_pose', default_value='0.0', description='Initial X position')
    declare_y_pose = DeclareLaunchArgument('y_pose', default_value='0.0', description='Initial Y position')
    declare_z_pose = DeclareLaunchArgument('z_pose', default_value='0.01', description='Initial Z position')
    declare_use_sim_time = DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation time')

    x_pose = LaunchConfiguration('x_pose')
    y_pose = LaunchConfiguration('y_pose')
    z_pose = LaunchConfiguration('z_pose')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Gazebo server & client
    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gazebo_ros_pkg, 'launch', 'gzserver.launch.py')),
        launch_arguments={'world': empty_world}.items()
    )

    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gazebo_ros_pkg, 'launch', 'gzclient.launch.py'))
    )

    # Robot spawn
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_entity',
        arguments=[
            '-entity', 'my_car',
            '-topic', 'robot_description',
            '-x', x_pose,
            '-y', y_pose,
            '-z', z_pose,
        ],
        output='screen'
    )

    # Controller Spawners
    joint_broad_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_broad', '--controller-manager', '/controller_manager'],
        parameters=[controller_config],
        output='screen'
    )

    diff_drive_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_cont', '--controller-manager', '/controller_manager'],
        parameters=[controller_config],
        output='screen'
    )

    # Twist Mux Node
    twist_mux_node = Node(
        package='twist_mux',
        executable='twist_mux',
        parameters=[twist_mux_config, {'use_sim_time': use_sim_time}],
        remappings=[('/cmd_vel_out', '/diff_cont/cmd_vel_unstamped')],
        output='screen'
    )

    return LaunchDescription([
        declare_x_pose,
        declare_y_pose,
        declare_z_pose,
        declare_use_sim_time,
        gzserver,
        gzclient,
        spawn_entity,
        joint_broad_spawner,
        diff_drive_spawner,
        twist_mux_node
    ])
