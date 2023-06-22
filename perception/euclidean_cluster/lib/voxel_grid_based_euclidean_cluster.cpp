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

#include "euclidean_cluster/voxel_grid_based_euclidean_cluster.hpp"

#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>

#include <unordered_map>

namespace euclidean_cluster
{
VoxelGridBasedEuclideanCluster::VoxelGridBasedEuclideanCluster()
{
}

VoxelGridBasedEuclideanCluster::VoxelGridBasedEuclideanCluster(
  bool use_height, int min_cluster_size, int max_cluster_size)
: EuclideanClusterInterface(use_height, min_cluster_size, max_cluster_size)
{
}

VoxelGridBasedEuclideanCluster::VoxelGridBasedEuclideanCluster(
  bool use_height, int min_cluster_size, int max_cluster_size, float tolerance,
  float voxel_leaf_size, int min_points_number_per_voxel)
: EuclideanClusterInterface(use_height, min_cluster_size, max_cluster_size),
  tolerance_(tolerance),
  voxel_leaf_size_(voxel_leaf_size),
  min_points_number_per_voxel_(min_points_number_per_voxel)
{
}

autoware_auto_perception_msgs::msg::DetectedObjects VoxelGridBasedEuclideanCluster::cluster(
  const pcl::PointCloud<pcl::PointXYZ>::ConstPtr & pointcloud,
  std::vector<pcl::PointCloud<pcl::PointXYZ>> & clusters)
{
  // TODO(Saito) implement use_height is false version

  // create voxel
  pcl::PointCloud<pcl::PointXYZ>::Ptr voxel_map_ptr(new pcl::PointCloud<pcl::PointXYZ>);
  voxel_grid_.setLeafSize(voxel_leaf_size_, voxel_leaf_size_, 100000.0);
  voxel_grid_.setMinimumPointsNumberPerVoxel(min_points_number_per_voxel_);
  voxel_grid_.setInputCloud(pointcloud);
  voxel_grid_.setSaveLeafLayout(true);
  voxel_grid_.filter(*voxel_map_ptr);

  // voxel is pressed 2d
  pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud_2d_ptr(new pcl::PointCloud<pcl::PointXYZ>);
  for (const auto & point : voxel_map_ptr->points) {
    pcl::PointXYZ point2d;
    point2d.x = point.x;
    point2d.y = point.y;
    point2d.z = 0.0;
    pointcloud_2d_ptr->push_back(point2d);
  }

  // create tree
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud(pointcloud_2d_ptr);

  // clustering
  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> pcl_euclidean_cluster;
  pcl_euclidean_cluster.setClusterTolerance(tolerance_);
  pcl_euclidean_cluster.setMinClusterSize(1);
  pcl_euclidean_cluster.setMaxClusterSize(max_cluster_size_);
  pcl_euclidean_cluster.setSearchMethod(tree);
  pcl_euclidean_cluster.setInputCloud(pointcloud_2d_ptr);
  pcl_euclidean_cluster.extract(cluster_indices);

  // create map to search cluster index from voxel grid index
  std::unordered_map</* voxel grid index */ int, /* cluster index */ int> map;
  for (size_t cluster_idx = 0; cluster_idx < cluster_indices.size(); ++cluster_idx) {
    const auto & cluster = cluster_indices.at(cluster_idx);
    for (const auto & point_idx : cluster.indices) {
      map[point_idx] = cluster_idx;
    }
  }

  // create vector of point cloud cluster. vector index is voxel grid index.
  std::vector<pcl::PointCloud<pcl::PointXYZ>> temporary_clusters;  // no check about cluster size
  temporary_clusters.resize(cluster_indices.size());
  for (const auto & point : pointcloud->points) {
    const int index =
      voxel_grid_.getCentroidIndexAt(voxel_grid_.getGridCoordinates(point.x, point.y, point.z));
    if (map.find(index) != map.end()) {
      temporary_clusters.at(map[index]).points.push_back(point);
    }
  }

  // define the autoware detected objects messages to be published
  autoware_auto_perception_msgs::msg::DetectedObjects objs;

  // build output and check cluster size
  {
    for (const auto & cluster : cluster_indices) {
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
      for (const auto & point_idx : cluster.indices) {
        cloud_cluster->points.push_back(pointcloud->points[point_idx]);
      }

      if (!(min_cluster_size_ <= static_cast<int>(cloud_cluster->points.size()) &&
            static_cast<int>(cloud_cluster->points.size()) <= max_cluster_size_)) {
        continue;
      }

      Eigen::Vector4f min_pt, max_pt;
      Eigen::Vector4f centroid;
      pcl::getMinMax3D(*cloud_cluster, min_pt, max_pt);

      pcl::compute3DCentroid(*cloud_cluster, centroid);

      Eigen::Vector3f dimensions = max_pt.head<3>() - min_pt.head<3>();

      float center_x = centroid.x();
      float center_y = centroid.y();
      float center_z = centroid.z();


      autoware_auto_perception_msgs::msg::DetectedObject obj;
      obj.kinematics.pose_with_covariance.pose.position.x = center_x;
      obj.kinematics.pose_with_covariance.pose.position.y = center_y;
      obj.kinematics.pose_with_covariance.pose.position.z = center_z;

      obj.kinematics.has_position_covariance = false;
      obj.kinematics.orientation_availability = autoware_auto_perception_msgs::msg::DetectedObjectKinematics::AVAILABLE;
      obj.kinematics.has_twist = false;
      obj.kinematics.has_twist_covariance = false;

      // Fill in the Polygon of the Object
      obj.shape.type = autoware_auto_perception_msgs::msg::Shape::POLYGON;
      obj.shape.footprint.points.resize(4);
      obj.shape.footprint.points[0].x = dimensions.x() / 2.0f;
      obj.shape.footprint.points[0].y = dimensions.y() / 2.0f;
      obj.shape.footprint.points[0].z = -5.0f;
      obj.shape.footprint.points[1].x = dimensions.x() / 2.0f;
      obj.shape.footprint.points[1].y = -dimensions.y() / 2.0f;
      obj.shape.footprint.points[1].z = -5.0f;
      obj.shape.footprint.points[2].x = -dimensions.x() / 2.0f;
      obj.shape.footprint.points[2].y = -dimensions.y() / 2.0f;
      obj.shape.footprint.points[2].z = -5.0f;
      obj.shape.footprint.points[3].x = -dimensions.x() / 2.0f;
      obj.shape.footprint.points[3].y = dimensions.y() / 2.0f;
      obj.shape.footprint.points[3].z = -5.0f;

      obj.shape.dimensions.z = dimensions.z() / 10;
      obj.shape.dimensions.x = dimensions.x();
      obj.shape.dimensions.y = dimensions.y();

      objs.objects.push_back(obj);

      clusters.push_back(*cloud_cluster);
      clusters.back().width = cloud_cluster->points.size();
      clusters.back().height = 1;
      clusters.back().is_dense = false;
    }
  }

  return objs;
}

}  // namespace euclidean_cluster
