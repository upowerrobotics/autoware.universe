# Copyright 2023 U Power Robotics, USA

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    pkg_share_path = get_package_share_directory("rtc_auto_mode_manager")

    # arguments
    param_path_arg = DeclareLaunchArgument(
        name="param_path",
        default_value=os.path.join(pkg_share_path, "config/rtc_auto_mode_manager.param.yaml"),
    )

    rtc_auto_mode_manager_node = Node(
        package="rtc_auto_mode_manager",
        executable="rtc_auto_mode_manager_node",
        name="rtc_auto_mode_manager",
        output="screen",
        parameters=[
            LaunchConfiguration("param_path")
        ],
    )

    return LaunchDescription([
        param_path_arg,
        rtc_auto_mode_manager_node
    ])
