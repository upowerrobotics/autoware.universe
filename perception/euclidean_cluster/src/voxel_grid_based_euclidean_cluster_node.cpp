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
  const bool use_height = this->declare_parameter("use_height", rclcpp::PARAMETER_BOOL).get<bool>();
  const int min_cluster_size = this->declare_parameter("min_cluster_size", rclcpp::PARAMETER_INTEGER).get<uint32_t>();
  const int max_cluster_size = this->declare_parameter("max_cluster_size", rclcpp::PARAMETER_INTEGER).get<uint32_t>();
  const float tolerance = this->declare_parameter("tolerance", rclcpp::PARAMETER_DOUBLE).get<float_t>();
  const float voxel_leaf_size = this->declare_parameter("voxel_leaf_size", rclcpp::PARAMETER_DOUBLE).get<float_t>();
  const int min_points_number_per_voxel = this->declare_parameter("min_points_number_per_voxel", rclcpp::PARAMETER_INTEGER).get<uint32_t>();
  cluster_ = std::make_shared<VoxelGridBasedEuclideanCluster>(
    use_height, min_cluster_size, max_cluster_size, tolerance, voxel_leaf_size,
    min_points_number_per_voxel);

  using std::placeholders::_1;
  pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "input", rclcpp::SensorDataQoS().keep_last(1),
    std::bind(&VoxelGridBasedEuclideanClusterNode::onPointCloud, this, _1));
  point_clusters_pub_ = this->create_publisher<autoware_auto_perception_msgs::msg::PointClusters>(
    "output", rclcpp::QoS{1});
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

  // construct PointCluster message from clusters
  autoware_auto_perception_msgs::msg::PointClusters point_clusters;
  convertPointCloudClusters2PointClusters(input_msg->header, clusters, point_clusters);
  point_clusters_pub_->publish(point_clusters);
}
}  // namespace euclidean_cluster

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(euclidean_cluster::VoxelGridBasedEuclideanClusterNode)