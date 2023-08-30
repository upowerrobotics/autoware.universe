# Copyright 2023 U Power Robotics, USA

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    mission_planner_share_path = get_package_share_directory("mission_planner")

    # arguments
    modified_goal_topic_name_arg = DeclareLaunchArgument(
        name="modified_goal_topic_name",
        default_value="/planning/scenario_planning/modified_goal",
    )

    map_topic_name_arg = DeclareLaunchArgument(
        name="map_topic_name",
        default_value="/map/vector_map",
        description="lanelet2 map topic name",
    )

    visualization_topic_name_arg = DeclareLaunchArgument(
        name="visualization_topic_name",
        default_value="/planning/mission_planning/route_marker",
    )

    mission_planner_param_path_arg = DeclareLaunchArgument(
        name="mission_planner_param_path",
        default_value=os.path.join(mission_planner_share_path, "config/mission_planner.param.yaml"),
    )

    mission_planner_node = Node(
        package="mission_planner",
        executable="mission_planner",
        name="mission_planner",
        output="screen",
        remappings=[
            ("input/modified_goal", LaunchConfiguration("modified_goal_topic_name")),
            ("input/vector_map", LaunchConfiguration("map_topic_name")),
            ("debug/route_marker", LaunchConfiguration("visualization_topic_name")),
        ],
        parameters=[
            LaunchConfiguration("mission_planner_param_path"),
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
            {"map_odom_topic": "/localization/absolute_odom"},
            {"map_frame": "map"},
        ],
    )

    return LaunchDescription([
        modified_goal_topic_name_arg,
        map_topic_name_arg,
        visualization_topic_name_arg,
        mission_planner_param_path_arg,
        mission_planner_node
    ])
