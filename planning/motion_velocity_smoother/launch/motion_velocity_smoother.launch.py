# Copyright 2023 U Power Robotics, USA

import os.path
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.actions import OpaqueFunction
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory


def launch_setup(context, *args, **kwargs):
    pkg_share_path = get_package_share_directory("motion_velocity_smoother")

    # arguments
    # common param
    common_param_path_arg = DeclareLaunchArgument(
        name="common_param_path",
        default_value=os.path.join(
            pkg_share_path, "config", "default_common.param.yaml"
        ),
    )

    nearest_search_param_path_arg = DeclareLaunchArgument(
        name="nearest_search_param_path",
    )

    # input/output
    input_trajectory_arg = DeclareLaunchArgument(
        name="input_trajectory",
        default_value="/planning/scenario_planning/scenario_selector/trajectory",
    )

    output_trajectory_arg = DeclareLaunchArgument(
        name="output_trajectory",
        default_value="/planning/scenario_planning/trajectory",
    )

    # debug flags
    publish_debug_trajs_arg = DeclareLaunchArgument(
        name="publish_debug_trajs",
        default_value="False",
    )

    smoother_type_arg = DeclareLaunchArgument(
        name="smoother_type",
        default_value="JerkFiltered",
        description="Analytical, JerkFiltered, L2, or Linf",
    )

    param_path_arg = DeclareLaunchArgument(
        name="param_path",
        default_value=os.path.join(
            pkg_share_path,
            "config", "default_motion_velocity_smoother.param.yaml",
        ),
    )

    smoother_param_path_arg = DeclareLaunchArgument(
        name="smoother_param_path",
        default_value=os.path.join(
            pkg_share_path, "config", LaunchConfiguration("smoother_type").perform(context),
            ".param.yaml"
        ),
    )

    motion_velocity_smoother_node = Node(
        package="motion_velocity_smoother",
        executable="motion_velocity_smoother",
        name="motion_velocity_smoother",
        output="screen",
        remappings=[
            ("~/input/trajectory", LaunchConfiguration("input_trajectory")),
            ("~/output/trajectory", LaunchConfiguration("output_trajectory")),
            ("~/input/external_velocity_limit_mps",
             "/planning/scenario_planning/max_velocity"),
            # Topic for setting maximum speed from the outside (input topic)
            ("~/output/current_velocity_limit_mps",
             "/planning/scenario_planning/current_max_velocity"),
            ("/localization/kinematic_state", LaunchConfiguration("odom_topic"))
        ],
        parameters=[
            LaunchConfiguration("common_param_path"),
            LaunchConfiguration("nearest_search_param_path"),
            LaunchConfiguration("param_path"),
            LaunchConfiguration("smoother_param_path"),
            {"publish_debug_trajs": LaunchConfiguration("publish_debug_trajs")},
            {"algorithm_type": LaunchConfiguration("smoother_type")},
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ]
    )

    return [
        common_param_path_arg,
        nearest_search_param_path_arg,
        input_trajectory_arg,
        output_trajectory_arg,
        publish_debug_trajs_arg,
        smoother_type_arg,
        param_path_arg,
        smoother_param_path_arg,
        motion_velocity_smoother_node
    ]


def generate_launch_description():
    return LaunchDescription([OpaqueFunction(function=launch_setup)])
