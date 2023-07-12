# Copyright 2023 U Power Robotics, USA
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import GroupAction
from launch_ros.actions import PushRosNamespace
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    pkg_share_path = get_package_share_directory("ad_api_adaptors")

    initial_pose_param_path = os.path.join(pkg_share_path, "config/initial_pose.param.yaml")

    with open(initial_pose_param_path, "r") as f:
        initial_pose_param = yaml.safe_load(f)["/**"]["ros__parameters"]

    group = GroupAction(
        actions=[
            PushRosNamespace("default_ad_api/helpers"),
            Node(
                package="ad_api_adaptors",
                executable="initial_pose_adaptor",
                name="initial_pose_adaptor",
                remappings=[
                    ("~/initialpose", "/initialpose"),
                    ("~/pointcloud_map", "/map/pointcloud_map"),
                    ("~/partial_map_load", "/map/get_partial_pointcloud_map"),
                ],
                parameters=[
                    {
                        "map_loader_name": "/map/pointcloud_map_loader",
                    },
                    initial_pose_param,
                ],
            ),

            Node(
                package="ad_api_adaptors",
                executable="routing_adaptor",
                name="routing_adaptor",
                remappings=[
                    ("~/input/fixed_goal", "/planning/mission_planning/goal"),
                    ("~/input/rough_goal", "/rviz/routing/rough_goal"),
                    ("~/input/waypoint", "/planning/mission_planning/checkpoint"),
                ]
            )
        ]
    )

    return LaunchDescription(
        [group]
    )

