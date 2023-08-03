# Copyright 2023 U Power Robotics, USA

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    pkg_share_path = get_package_share_directory("planning_validator")

    # arguments
    planning_validator_param_path_arg = DeclareLaunchArgument(
        name="planning_validator_param_path",
        default_value=os.path.join(
            pkg_share_path,
            "config/planning_validator.param.yaml"
        ),
    )

    input_trajectory_arg = DeclareLaunchArgument(
        name="input_trajectory",
        default_value="/planning/scenario_planning/motion_velocity_smoother/trajectory",
    )

    output_trajectory_arg = DeclareLaunchArgument(
        name="output_trajectory",
        default_value="/planning/scenario_planning/trajectory"
    )

    planning_validator_node = Node(
        package="planning_validator",
        executable="planning_validator_node",
        name="planning_validator",
        output="screen",
        remappings=[
            ("~/input/trajectory", LaunchConfiguration("input_trajectory")),
            ("~/input/kinematics", LaunchConfiguration("odom_topic_name")),
            ("~/output/trajectory", LaunchConfiguration("output_trajectory")),
            ("~/output/validation_status", "~/validation_status"),
        ],
        parameters=[
            LaunchConfiguration("planning_validator_param_path"),
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ],
    )

    return LaunchDescription([
        planning_validator_param_path_arg,
        input_trajectory_arg,
        output_trajectory_arg,
        planning_validator_node
    ])
