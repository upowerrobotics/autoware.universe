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
    autoware_auto_perception_msgs::msg::DetectedObject detected_object;

    auto cluster_ptr = boost::make_shared<const pcl::PointCloud<pcl::PointXYZ>>(cluster);

    // create moment of inertia feature extractor
    pcl::MomentOfInertiaEstimation <pcl::PointXYZ> feature_extractor;


    feature_extractor.setInputCloud(cluster_ptr);
    feature_extractor.compute();

    std::vector<float> moment_of_inertia;
    std::vector<float> eccentricity;

    pcl::PointXYZ min_point_OBB;
    pcl::PointXYZ max_point_OBB;
    pcl::PointXYZ position_OBB;
    Eigen::Matrix3f rotational_matrix_OBB;
    Eigen::Vector3f mass_center;
    
    // compute the moment of inertia
    feature_extractor.getMomentOfInertia(moment_of_inertia);
    // compute eccentricity
    feature_extractor.getEccentricity(eccentricity);
    // get oriented bounding box
    feature_extractor.getOBB(min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);
    // get the mass center
    feature_extractor.getMassCenter(mass_center);

    Eigen::Vector3f position(position_OBB.x, position_OBB.y, position_OBB.z);
    Eigen::Quaternionf quat(rotational_matrix_OBB);

    // compute the cluster dimension
    float dim_x = max_point_OBB.x - min_point_OBB.x;
    float dim_y = max_point_OBB.y - min_point_OBB.y;
    float dim_z = max_point_OBB.z - min_point_OBB.z;

    if (dim_z > 5.0f || dim_z < 1.0f
      || dim_x > 20.0f ||dim_y > 20.0f
      || dim_x * dim_y / dim_z > 10.0f)
      continue;

    Eigen::Vector3f p1(dim_x / 2.0f, dim_y / 2.0f, 0.0f);
    Eigen::Vector3f p2(dim_x / 2.0f, -dim_y / 2.0f, 0.0f);
    Eigen::Vector3f p3(-dim_x / 2.0f, -dim_y / 2.0f, 0.0f);
    Eigen::Vector3f p4(-dim_x / 2.0f, dim_y / 2.0f, 0.0f);

    Eigen::Vector3f p1_rot = quat * p1;
    Eigen::Vector3f p2_rot = quat * p2;

    geometry_msgs::msg::Point centroid;
    centroid.x = position.x();
    centroid.y = position.y();
    centroid.z = position.z();

    detected_object.kinematics.pose_with_covariance.pose.position = centroid;
    detected_object.kinematics.has_position_covariance = false;
    detected_object.kinematics.orientation_availability = autoware_auto_perception_msgs::msg::DetectedObjectKinematics::AVAILABLE;
    detected_object.kinematics.has_twist = false;
    detected_object.kinematics.has_twist_covariance = false;

    // Fill in the Polygon of the Object
    detected_object.shape.type = autoware_auto_perception_msgs::msg::Shape::POLYGON;
    detected_object.shape.footprint.points.resize(4);
    detected_object.shape.footprint.points[0].x = p1_rot.x();
    detected_object.shape.footprint.points[0].y = p1_rot.y();
    detected_object.shape.footprint.points[0].z = 0.0f;
    detected_object.shape.footprint.points[1].x = p2_rot.x();
    detected_object.shape.footprint.points[1].y = p2_rot.y();
    detected_object.shape.footprint.points[1].z = 0.0f;
    detected_object.shape.footprint.points[2].x = -p1_rot.x();
    detected_object.shape.footprint.points[2].y = -p1_rot.y();
    detected_object.shape.footprint.points[2].z = 0.0f;
    detected_object.shape.footprint.points[3].x = -p2_rot.x();
    detected_object.shape.footprint.points[3].y = -p2_rot.y();
    detected_object.shape.footprint.points[3].z = 0.0f;
    detected_object.shape.dimensions.x = dim_x;
    detected_object.shape.dimensions.y = dim_y;
    detected_object.shape.dimensions.z = dim_z;
    // add the detected object to detected objects message
    msg.objects.emplace_back(detected_object);
  }
}
}  // namespace euclidean_cluster