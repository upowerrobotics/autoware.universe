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

#include "euclidean_cluster/euclidean_cluster.hpp"

#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>

namespace euclidean_cluster
{
EuclideanCluster::EuclideanCluster()
{
}

EuclideanCluster::EuclideanCluster(bool use_height, int min_cluster_size, int max_cluster_size)
: EuclideanClusterInterface(use_height, min_cluster_size, max_cluster_size)
{
}

EuclideanCluster::EuclideanCluster(
  bool use_height, int min_cluster_size, int max_cluster_size, float tolerance)
: EuclideanClusterInterface(use_height, min_cluster_size, max_cluster_size), tolerance_(tolerance)
{
}

autoware_auto_perception_msgs::msg::DetectedObjects EuclideanCluster::cluster(
  const pcl::PointCloud<pcl::PointXYZ>::ConstPtr & pointcloud,
  std::vector<pcl::PointCloud<pcl::PointXYZ>> & clusters)
{
  // convert 2d pointcloud
  pcl::PointCloud<pcl::PointXYZ>::ConstPtr pointcloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
  if (!use_height_) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud_2d_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    for (const auto & point : pointcloud->points) {
      pcl::PointXYZ point2d;
      point2d.x = point.x;
      point2d.y = point.y;
      point2d.z = 0.0;
      pointcloud_2d_ptr->push_back(point2d);
    }
    pointcloud_ptr = pointcloud_2d_ptr;
  } else {
    pointcloud_ptr = pointcloud;
  }

  // create tree
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud(pointcloud_ptr);

  // clustering
  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<pcl::PointXYZ> pcl_euclidean_cluster;
  pcl_euclidean_cluster.setClusterTolerance(tolerance_);
  pcl_euclidean_cluster.setMinClusterSize(min_cluster_size_);
  pcl_euclidean_cluster.setMaxClusterSize(max_cluster_size_);
  pcl_euclidean_cluster.setSearchMethod(tree);
  pcl_euclidean_cluster.setInputCloud(pointcloud_ptr);
  pcl_euclidean_cluster.extract(cluster_indices);

  // define the autoware detected objects messages to be published
  autoware_auto_perception_msgs::msg::DetectedObjects objs;

  // build output
  {
    for (const auto & cluster : cluster_indices) {
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
      for (const auto & point_idx : cluster.indices) {
        cloud_cluster->points.push_back(pointcloud->points[point_idx]);
      }

      Eigen::Vector4f min_pt, max_pt;
      Eigen::Vector4f centroid;
      pcl::getMinMax3D(*cloud_cluster, min_pt, max_pt);

      pcl::compute3DCentroid(*cloud_cluster, centroid);

      Eigen::Vector3f dimensions = max_pt.head<3>() - min_pt.head<3>();

      float center_x = centroid.x();
      float center_y = centroid.y();
      float center_z = centroid.z();

      if (dimensions.z() < 5.0f && dimensions.z() > 1.0f
      && dimensions.x() < 10.0f && dimensions.y() < 10.0f && center_z < 2.0f
      && center_x * center_x + center_y * center_y < 1000.0f
      && dimensions.x() * dimensions.y() * dimensions.z() < 20.0f)
      {
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
        obj.shape.footprint.points[0].z = 0.0f;
        obj.shape.footprint.points[1].x = dimensions.x() / 2.0f;
        obj.shape.footprint.points[1].y = -dimensions.y() / 2.0f;
        obj.shape.footprint.points[1].z = 0.0f;
        obj.shape.footprint.points[2].x = -dimensions.x() / 2.0f;
        obj.shape.footprint.points[2].y = -dimensions.y() / 2.0f;
        obj.shape.footprint.points[2].z = 0.0f;
        obj.shape.footprint.points[3].x = -dimensions.x() / 2.0f;
        obj.shape.footprint.points[3].y = dimensions.y() / 2.0f;
        obj.shape.footprint.points[3].z = 0.0f;

        obj.shape.dimensions.z = dimensions.z();
        obj.shape.dimensions.x = dimensions.x();
        obj.shape.dimensions.y = dimensions.y();

        objs.objects.push_back(obj);

        clusters.push_back(*cloud_cluster);
        clusters.back().width = cloud_cluster->points.size();
        clusters.back().height = 1;
        clusters.back().is_dense = false;
      }
    }
  }
  return objs;
}

}  // namespace euclidean_cluster
