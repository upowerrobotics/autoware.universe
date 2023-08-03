# Copyright 2022 TIER IV, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import launch
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
import os
from ament_index_python.packages import get_package_share_directory
import yaml


def create_api_node(node_name, class_name, **kwargs):
    config_file = os.path.join(get_package_share_directory("default_ad_api"), "config", "default_ad_api.param.yaml")
    with open(config_file, 'r') as file:
        config_params = yaml.safe_load(file)["/**"]["ros__parameters"]

    return ComposableNode(
        namespace="default_ad_api/node",
        name=node_name,
        package="default_ad_api",
        plugin="default_ad_api::" + class_name,
        parameters=[config_params],
    )


def generate_launch_description():
    components = [
        create_api_node("routing", "RoutingNode"),
    ]
    container = ComposableNodeContainer(
        namespace="default_ad_api",
        name="container",
        package="rclcpp_components",
        executable="component_container_mt",
        composable_node_descriptions=components,
    )
    return launch.LaunchDescription([container])
