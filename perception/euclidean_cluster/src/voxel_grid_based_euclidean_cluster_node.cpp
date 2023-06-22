// Copyright 2020 Tier IV, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "voxel_grid_based_euclidean_cluster_node.hpp"

#include "euclidean_cluster/utils.hpp"

#include <vector>

namespace euclidean_cluster
{
VoxelGridBasedEuclideanClusterNode::VoxelGridBasedEuclideanClusterNode(
  const rclcpp::NodeOptions & options)
: Node("voxel_grid_based_euclidean_cluster_node", options)
{
  const bool use_height = this->declare_parameter("use_height", false);
  const int min_cluster_size = this->declare_parameter("min_cluster_size", 1);
  const int max_cluster_size = this->declare_parameter("max_cluster_size", 1024);
  const float tolerance = this->declare_parameter("tolerance", 1.0);
  const float voxel_leaf_size = this->declare_parameter("voxel_leaf_size", 0.01);
  const int min_points_number_per_voxel = this->declare_parameter("min_points_number_per_voxel", 3);
  cluster_ = std::make_shared<VoxelGridBasedEuclideanCluster>(
    use_height, min_cluster_size, max_cluster_size, tolerance, voxel_leaf_size,
    min_points_number_per_voxel);

  detected_objects_pub_ = this->create_publisher<autoware_auto_perception_msgs::msg::DetectedObjects>(
    "detected_objects_universe", rclcpp::QoS{1});

  using std::placeholders::_1;
  pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "input", rclcpp::SensorDataQoS().keep_last(1),
    std::bind(&VoxelGridBasedEuclideanClusterNode::onPointCloud, this, _1));
}

void VoxelGridBasedEuclideanClusterNode::onPointCloud(
  const sensor_msgs::msg::PointCloud2::ConstSharedPtr input_msg)
{
  // convert ros to pcl
  pcl::PointCloud<pcl::PointXYZ>::Ptr raw_pointcloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*input_msg, *raw_pointcloud_ptr);

  // clustering
  std::vector<pcl::PointCloud<pcl::PointXYZ>> clusters;
  cluster_->cluster(raw_pointcloud_ptr, clusters);
  autoware_auto_perception_msgs::msg::DetectedObjects detected_objects = cluster_->cluster(raw_pointcloud_ptr, clusters);
  detected_objects.header.frame_id = input_msg->header.frame_id;
  detected_objects.header.stamp = input_msg->header.stamp;
  detected_objects_pub_->publish(detected_objects);

}

}  // namespace euclidean_cluster

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(euclidean_cluster::VoxelGridBasedEuclideanClusterNode)
