#!/usr/bin/env python3

import os
import xacro

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Paths to robot_description package resources
    robot_desc_pkg = get_package_share_directory('four_wheels_robot_description')
    rviz_config = os.path.join(robot_desc_pkg, 'rviz', 'car.rviz')
    # controller_config = os.path.join(robot_desc_pkg, 'config', 'my_controllers.yaml')

    # Launch arguments
    gui_arg = DeclareLaunchArgument('gui', default_value='true', choices=['true', 'false'],
                                   description='Run joint_state_publisher_gui')
    use_sim_time_arg = DeclareLaunchArgument('use_sim_time', default_value='false',
                                            description='Use simulation time')

    gui = LaunchConfiguration('gui')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Process URDF with xacro
    xacro_file = os.path.join(robot_desc_pkg, 'urdf', 'robot.urdf')
    robot_desc_xml = xacro.process_file(xacro_file).toxml()
    robot_description_param = {'robot_description': ParameterValue(robot_desc_xml, value_type=str)}

    # Nodes
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description_param, {'use_sim_time': use_sim_time}]
    )

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        output='screen',
        condition=UnlessCondition(gui),
        parameters=[{'use_sim_time': use_sim_time}]
    )

    joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        output='screen',
        condition=IfCondition(gui),
        parameters=[{'use_sim_time': use_sim_time}]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    return LaunchDescription([
        gui_arg,
        use_sim_time_arg,
        robot_state_publisher,
        # joint_state_publisher,
        # joint_state_publisher_gui,
        rviz_node,
    ])
