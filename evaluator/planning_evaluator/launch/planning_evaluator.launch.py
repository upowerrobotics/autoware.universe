# Copyright 2023 U Power Robotics, USA

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    pkg_share_path = get_package_share_directory("planning_evaluator")

    # arguments
    input_odometry_arg = DeclareLaunchArgument(
        name="input/odometry",
        default_value="/localization/kinematic_state",
    )

    input_trajectory_arg = DeclareLaunchArgument(
        name="input/trajectory",
        default_value="/planning/scenario_planning/trajectory",
    )

    input_reference_trajectory_arg = DeclareLaunchArgument(
        name="input/reference_trajectory",
        default_value="/planning/scenario_planning/lane_driving/motion_planning/obstacle_avoidance_planner/trajectory",
    )

    input_objects_arg = DeclareLaunchArgument(
        name="input/objects",
        default_value="/perception/object_recognition/objects",
    )

    input_modified_goal_arg = DeclareLaunchArgument(
        name="input/modified_goal",
        default_value="/planning/scenario_planning/modified_goal",
    )

    use_sim_time_arg = DeclareLaunchArgument(
        name="use_sim_time",
        default_value="False",
        description="Use simulated time",
    )

    planning_evaluator_node = Node(
        package="planning_evaluator",
        executable="planning_evaluator",
        name="planning_evaluator",
        output="screen",
        remappings=[
            ("~/input/odometry", LaunchConfiguration("input/odometry")),
            ("~/input/trajectory", LaunchConfiguration("input/trajectory")),
            ("~/input/reference_trajectory", LaunchConfiguration("input/reference_trajectory")),
            ("~/input/objects", LaunchConfiguration("input/objects")),
            ("~/input/modified_goal", LaunchConfiguration("input/modified_goal")),
            ("~/metrics", "/diagnostic/planning_evaluator/metrics"),
        ],
        parameters=[
            os.path.join(
                pkg_share_path,
                "param/planning_evaluator.defaults.yaml"
            ),
            {"use_sim_time_arg", LaunchConfiguration("use_sim_time_arg")},
        ],
    )

    return LaunchDescription([
        input_odometry_arg,
        input_trajectory_arg,
        input_reference_trajectory_arg,
        input_objects_arg,
        input_modified_goal_arg,
        use_sim_time_arg,
        planning_evaluator_node
    ])
