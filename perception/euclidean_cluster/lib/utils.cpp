// Copyright 2021 Tier IV, Inc.
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
#include "euclidean_cluster/utils.hpp"

#include <autoware_auto_perception_msgs/msg/object_classification.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <tier4_perception_msgs/msg/detected_object_with_feature.hpp>
#include <tier4_perception_msgs/msg/detected_objects_with_feature.hpp>
#include <autoware_auto_perception_msgs/msg/detected_object.hpp>
#include <autoware_auto_perception_msgs/msg/detected_objects.hpp>

namespace euclidean_cluster
{
geometry_msgs::msg::Point getCentroid(const sensor_msgs::msg::PointCloud2 & pointcloud)
{
  geometry_msgs::msg::Point centroid;
  centroid.x = 0.0f;
  centroid.y = 0.0f;
  centroid.z = 0.0f;
  for (sensor_msgs::PointCloud2ConstIterator<float> iter_x(pointcloud, "x"),
       iter_y(pointcloud, "y"), iter_z(pointcloud, "z");
       iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
    centroid.x += *iter_x;
    centroid.y += *iter_y;
    centroid.z += *iter_z;
  }
  const size_t size = pointcloud.width * pointcloud.height;
  centroid.x = centroid.x / static_cast<float>(size);
  centroid.y = centroid.y / static_cast<float>(size);
  centroid.z = centroid.z / static_cast<float>(size);
  return centroid;
}

void convertPointCloudClusters2Msg(
  const std_msgs::msg::Header & header,
  const std::vector<pcl::PointCloud<pcl::PointXYZ>> & clusters,
  tier4_perception_msgs::msg::DetectedObjectsWithFeature & msg)
{
  msg.header = header;
  for (const auto & cluster : clusters) {
    sensor_msgs::msg::PointCloud2 ros_pointcloud;
    tier4_perception_msgs::msg::DetectedObjectWithFeature feature_object;
    pcl::toROSMsg(cluster, ros_pointcloud);
    ros_pointcloud.header = header;
    feature_object.feature.cluster = ros_pointcloud;
    feature_object.object.kinematics.pose_with_covariance.pose.position =
      getCentroid(ros_pointcloud);
    autoware_auto_perception_msgs::msg::ObjectClassification classification;
    classification.label = autoware_auto_perception_msgs::msg::ObjectClassification::UNKNOWN;
    classification.probability = 1.0f;
    feature_object.object.classification.emplace_back(classification);
    msg.feature_objects.push_back(feature_object);
  }
}
void convertObjectMsg2SensorMsg(
  const tier4_perception_msgs::msg::DetectedObjectsWithFeature & input,
  sensor_msgs::msg::PointCloud2 & output)
{
  output.header = input.header;

  size_t pointcloud_size = 0;
  for (const auto & feature_object : input.feature_objects) {
    pointcloud_size += feature_object.feature.cluster.width * feature_object.feature.cluster.height;
  }

  sensor_msgs::PointCloud2Modifier modifier(output);
  modifier.setPointCloud2Fields(
    4, "x", 1, sensor_msgs::msg::PointField::FLOAT32, "y", 1, sensor_msgs::msg::PointField::FLOAT32,
    "z", 1, sensor_msgs::msg::PointField::FLOAT32, "rgb", 1, sensor_msgs::msg::PointField::FLOAT32);
  modifier.resize(pointcloud_size);

  sensor_msgs::PointCloud2Iterator<float> iter_out_x(output, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_out_y(output, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_out_z(output, "z");
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_out_r(output, "r");
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_out_g(output, "g");
  sensor_msgs::PointCloud2Iterator<uint8_t> iter_out_b(output, "b");

  constexpr uint8_t color_data[] = {200, 0,   0, 0,   200, 0,   0, 0,   200,
                                    200, 200, 0, 200, 0,   200, 0, 200, 200};  // 6 pattern
  for (size_t i = 0; i < input.feature_objects.size(); ++i) {
    const auto & feature_object = input.feature_objects.at(i);
    sensor_msgs::PointCloud2ConstIterator<float> iter_in_x(feature_object.feature.cluster, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_in_y(feature_object.feature.cluster, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_in_z(feature_object.feature.cluster, "z");
    for (; iter_in_x != iter_in_x.end(); ++iter_in_x, ++iter_in_y, ++iter_in_z, ++iter_out_x,
                                         ++iter_out_y, ++iter_out_z, ++iter_out_r, ++iter_out_g,
                                         ++iter_out_b) {
      *iter_out_x = *iter_in_x;
      *iter_out_y = *iter_in_y;
      *iter_out_z = *iter_in_z;
      *iter_out_r = color_data[3 * (i % 6) + 0];
      *iter_out_g = color_data[3 * (i % 6) + 1];
      *iter_out_b = color_data[3 * (i % 6) + 2];
    }
  }

  output.width = pointcloud_size;
  output.height = 1;
  output.is_dense = false;
}

void convertPointCloudClusters2DetectedObjects(
  const std_msgs::msg::Header & header,
  const std::vector<pcl::PointCloud<pcl::PointXYZ>> & clusters,
  autoware_auto_perception_msgs::msg::DetectedObjects & msg)
{
  msg.header = header;
  for (const auto & cluster : clusters) {
    sensor_msgs::msg::PointCloud2 ros_pointcloud;
    autoware_auto_perception_msgs::msg::DetectedObject detected_object;
    pcl::toROSMsg(cluster, ros_pointcloud);
    ros_pointcloud.header = header;
    detected_object.kinematics.pose_with_covariance.pose.position =
      getCentroid(ros_pointcloud);
    // find out the dimension of the bounding box
    Eigen::Vector4f min_pt, max_pt;
    pcl::getMinMax3D(cluster, min_pt, max_pt);
    Eigen::Vector3f dimensions = max_pt.head<3>() - min_pt.head<3>();

    if (dimensions.z() > 5.0f || dimensions.z() < 1.0f
      || dimensions.x() > 20.0f || dimensions.y() > 20.0f
      || dimensions.x() * dimensions.y() / dimensions.z() > 10.0f
      || detected_object.kinematics.pose_with_covariance.pose.position.z > 5.0f)
      continue;

    // get eigen vectors and eigen clusters of the point clusters
    Eigen::Vector4f pcaCentroid;
    pcl::compute3DCentroid(cluster, pcaCentroid);
    Eigen::Matrix3f covariance;
    pcl::computeCovarianceMatrixNormalized(cluster, pcaCentroid, covariance);
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigen_solver(covariance, Eigen::ComputeEigenvectors);
    Eigen::Matrix3f eigenVectorsPCA = eigen_solver.eigenvectors();
    Eigen::Vector3f eigenValuesPCA = eigen_solver.eigenvalues();
    eigenVectorsPCA.col(2) = eigenVectorsPCA.col(0).cross(eigenVectorsPCA.col(1));
    eigenVectorsPCA.col(0) = eigenVectorsPCA.col(1).cross(eigenVectorsPCA.col(2));
    eigenVectorsPCA.col(1) = eigenVectorsPCA.col(2).cross(eigenVectorsPCA.col(0));
    std::cout << "va (3x1):\n" << eigenValuesPCA << std::endl;
    std::cout << "ve (3x3):\n" << eigenVectorsPCA << std::endl;
    std::cout << "centroid (4x1):\n" << pcaCentroid << std::endl;

    // construct transform and inverse transform matrices
    Eigen::Matrix4f tm = Eigen::Matrix4f::Identity();
    Eigen::Matrix4f tm_inv = Eigen::Matrix4f::Identity();
    tm.block<3, 3>(0, 0) = eigenVectorsPCA.transpose();   //R.
    tm.block<3, 1>(0, 3) = -1.0f * (eigenVectorsPCA.transpose()) *(pcaCentroid.head<3>());//  -R*t
    tm_inv = tm.inverse();

    pcl::PointCloud<pcl::PointXYZ>::Ptr transformedCloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(cluster, *transformedCloud, tm);

    pcl::PointXYZ min_p1, max_p1;
    Eigen::Vector3f c1, c;
    pcl::getMinMax3D(*transformedCloud, min_p1, max_p1);
    c1 = 0.5f*(min_p1.getVector3fMap() + max_p1.getVector3fMap());

    std::cout << "centroid (3x1):\n" << c1 << std::endl;

    Eigen::Affine3f tm_inv_aff(tm_inv);
    pcl::transformPoint(c1, c, tm_inv_aff);

    Eigen::Vector3f whd, whd1;
    whd1 = max_p1.getVector3fMap() - min_p1.getVector3fMap();
    whd = whd1;

    std::cout << "width1=" << whd1(0) << std::endl;
    std::cout << "heght1=" << whd1(1) << std::endl;
    std::cout << "depth1=" << whd1(2) << std::endl;

    // construct rectangular vertices
    pcl::PointCloud<pcl::PointXYZ>::Ptr transVertexCloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr VertexCloud(new pcl::PointCloud<pcl::PointXYZ>);

    transVertexCloud->width = 6;
    transVertexCloud->height = 1;
    transVertexCloud->is_dense = false;
    transVertexCloud->points.resize(transVertexCloud->width * transVertexCloud->height);
    transVertexCloud->points[0].x = max_p1.x;
    transVertexCloud->points[0].y = max_p1.y;
    transVertexCloud->points[0].z = max_p1.z;
    transVertexCloud->points[1].x = max_p1.x;
    transVertexCloud->points[1].y = max_p1.y;
    transVertexCloud->points[1].z = min_p1.z;
    transVertexCloud->points[2].x = max_p1.x;
    transVertexCloud->points[2].y = min_p1.y;
    transVertexCloud->points[2].z = min_p1.z;
    transVertexCloud->points[3].x = min_p1.x;
    transVertexCloud->points[3].y = max_p1.y;
    transVertexCloud->points[3].z = max_p1.z;
    transVertexCloud->points[4].x = min_p1.x;
    transVertexCloud->points[4].y = min_p1.y;
    transVertexCloud->points[4].z = max_p1.z;
    transVertexCloud->points[5].x = min_p1.x;
    transVertexCloud->points[5].y = min_p1.y;
    transVertexCloud->points[5].z = min_p1.z;
    pcl::transformPointCloud(*transVertexCloud, *VertexCloud, tm_inv);



    detected_object.kinematics.has_position_covariance = false;
    detected_object.kinematics.orientation_availability = autoware_auto_perception_msgs::msg::DetectedObjectKinematics::AVAILABLE;
    detected_object.kinematics.has_twist = false;
    detected_object.kinematics.has_twist_covariance = false;

    // Fill in the Polygon of the Object
    detected_object.shape.type = autoware_auto_perception_msgs::msg::Shape::POLYGON;
    detected_object.shape.footprint.points.resize(4);
    detected_object.shape.footprint.points[0].x = (VertexCloud->points[0].x - VertexCloud->points[5].x) / 2.0f;
    detected_object.shape.footprint.points[0].y = (VertexCloud->points[0].y - VertexCloud->points[5].y) / 2.0f;
    detected_object.shape.footprint.points[0].z = 0.0f;
    detected_object.shape.footprint.points[1].x = (VertexCloud->points[2].x - VertexCloud->points[3].x) / 2.0f;
    detected_object.shape.footprint.points[1].y = -(VertexCloud->points[2].y - VertexCloud->points[3].y) / 2.0f;
    detected_object.shape.footprint.points[1].z = 0.0f;
    detected_object.shape.footprint.points[2].x = -(VertexCloud->points[0].x - VertexCloud->points[5].x) / 2.0f;
    detected_object.shape.footprint.points[2].y = -(VertexCloud->points[0].y - VertexCloud->points[5].y) / 2.0f;
    detected_object.shape.footprint.points[2].z = 0.0f;
    detected_object.shape.footprint.points[3].x = -(VertexCloud->points[2].x - VertexCloud->points[3].x) / 2.0f;
    detected_object.shape.footprint.points[3].y = (VertexCloud->points[2].y - VertexCloud->points[3].y) / 2.0f;
    detected_object.shape.footprint.points[3].z = 0.0f;
    detected_object.shape.dimensions.z = whd(0);
    detected_object.shape.dimensions.x = whd(1);
    detected_object.shape.dimensions.y = whd(2);
    // add the detected object to detected objects message
    msg.objects.emplace_back(detected_object);
  }
}
}  // namespace euclidean_cluster
