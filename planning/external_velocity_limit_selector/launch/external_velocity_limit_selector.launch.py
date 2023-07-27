# Copyright 2023 U Power Robotics, USA

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_path
import os


def generate_launch_description():

    pkg_share_path = get_package_share_path("external_velocity_limit_selector")
    # arguments
    # common param
    common_param_path_arg = DeclareLaunchArgument(
        name="common_param_path",
        default_value=os.path.join(
            pkg_share_path,
            "config/default_common.param.yaml",
        )
    )

    param_path_arg = DeclareLaunchArgument(
        name="param_path",
        default_value=os.path.join(
            pkg_share_path,
            "config/default.param.yaml",
        )
    )

    # input/output
    input_velocity_limit_from_api_arg = DeclareLaunchArgument(
        name="input_velocity_limit_from_api",
        default_value="/planning/scenario_planning/max_velocity_default",
    )

    input_velocity_limit_from_internal_arg = DeclareLaunchArgument(
        name="input_velocity_limit_from_internal",
        default_value="/planning/scenario_planning/max_velocity_candidates",
    )

    input_velocity_limit_clear_command_from_internal_arg = DeclareLaunchArgument(
        name="input_velocity_limit_clear_command_from_internal",
        default_value="/planning/scenario_planning/clear_velocity_limit",
    )

    output_velocity_limit_from_selector_arg = DeclareLaunchArgument(
        name="output_velocity_limit_from_selector",
        default_value="/planning/scenario_planning/max_velocity",
    )

    output_debug_string_arg = DeclareLaunchArgument(
        name="output_debug_string",
        default_value="/planning/scenario_planning/external_velocity_limit_selector/debug",
    )

    external_velocity_limit_selector_node = Node(
        package="external_velocity_limit_selector",
        executable="external_velocity_limit_selector",
        name="external_velocity_limit_selector",
        output="screen",
        remappings=[
            ("input/velocity_limit_from_api",
             LaunchConfiguration("input_velocity_limit_from_api")),
            ("input/velocity_limit_from_internal",
             LaunchConfiguration("input_velocity_limit_from_internal")),
            ("input/velocity_limit_clear_command_from_internal",
             LaunchConfiguration("input_velocity_limit_clear_command_from_internal")),
            ("output/external_velocity_limit",
             LaunchConfiguration("output_velocity_limit_from_selector")),
            ("output/debug", LaunchConfiguration("output_debug_string")),
        ],
        parameters=[
            LaunchConfiguration("common_param_path"),
            LaunchConfiguration("param_path"),
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
    )

    return LaunchDescription([
        common_param_path_arg,
        param_path_arg,
        input_velocity_limit_from_api_arg,
        input_velocity_limit_from_internal_arg,
        input_velocity_limit_clear_command_from_internal_arg,
        output_velocity_limit_from_selector_arg,
        output_debug_string_arg,
        external_velocity_limit_selector_node,
    ])
