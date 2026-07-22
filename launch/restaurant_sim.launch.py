#!/usr/bin/env python3
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    pkg_share = get_package_share_directory("restaurant_sim")
    
    gui = LaunchConfiguration("gui")
    world_name = LaunchConfiguration("world_name")

    declared_args = [
        DeclareLaunchArgument("gui", default_value="true", description="Launch Gazebo client GUI"),
        DeclareLaunchArgument("world_name", default_value="restaurant.sdf", description="World file name: restaurant.sdf or restaurant_empty_test.sdf"),
    ]

    world_path = PathJoinSubstitution([pkg_share, "worlds", world_name])

    gzserver = ExecuteProcess(
        cmd=[
            "gzserver", "--verbose", world_path,
            "-s", "libgazebo_ros_init.so",
            "-s", "libgazebo_ros_factory.so",
        ],
        output="screen",
    )

    gzclient = ExecuteProcess(
        cmd=["gzclient"],
        output="screen",
        condition=IfCondition(gui),
    )

    return LaunchDescription(declared_args + [gzserver, gzclient])
