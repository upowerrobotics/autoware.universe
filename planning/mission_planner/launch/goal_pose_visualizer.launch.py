# Copyright 2023 U Power Robotics, USA

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    # arguments
    route_topic_name_arg = DeclareLaunchArgument(
        name="route_topic_name",
        default_value="/planning/mission_planning/route",
    )

    echo_back_goal_pose_topic_name_arg = DeclareLaunchArgument(
        name="echo_back_goal_pose_topic_name",
        default_value="/planning/mission_planning/echo_back_goal_pose",
    )

    goal_pose_visualizer_node = Node(
        package="mission_planner",
        executable="goal_pose_visualizer",
        name="goal_pose_visualizer",
        output="screen",
        remappings=[
            ("input/route", LaunchConfiguration("route_topic_name")),
            ("output/goal_pose", LaunchConfiguration("echo_back_goal_pose_topic_name")),
        ],
        parameters=[
            {"use_sim_time": LaunchConfiguration("use_sim_time")},
        ]
    )

    return LaunchDescription([
        route_topic_name_arg,
        echo_back_goal_pose_topic_name_arg,
        goal_pose_visualizer_node
    ])
