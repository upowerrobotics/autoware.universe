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

#pragma once

#include "euclidean_cluster/voxel_grid_based_euclidean_cluster.hpp"

#include <rclcpp/rclcpp.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tier4_perception_msgs/msg/detected_objects_with_feature.hpp>

#include <memory>

namespace euclidean_cluster
{
class VoxelGridBasedEuclideanClusterNode : public rclcpp::Node
{
public:
  explicit VoxelGridBasedEuclideanClusterNode(const rclcpp::NodeOptions & options);

private:
  void onPointCloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr input_msg);

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
  rclcpp::Publisher<autoware_auto_perception_msgs::msg::PointClusters>::SharedPtr point_clusters_pub_;

  std::shared_ptr<VoxelGridBasedEuclideanCluster> cluster_;
};

}  // namespace euclidean_cluster