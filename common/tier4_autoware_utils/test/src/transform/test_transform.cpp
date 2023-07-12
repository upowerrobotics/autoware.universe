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

#include "tier4_autoware_utils/transform/transforms.hpp"

#include <gtest/gtest.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <thread>

TEST(system, transform_point_cloud)
{
  pcl::PointCloud<pcl::PointXYZI> cloud;
  pcl::PointXYZI pt_1, pt_2;
  pt_1.x = 10.055880;
  pt_1.y = -42.758892;
  pt_1.z = -10.636949;
  pt_1.intensity = 4;
  cloud.push_back(pt_1);
  pt_2.x = 23.282284;
  pt_2.y = -29.485722;
  pt_2.z = -11.468469;
  pt_2.intensity = 5;
  cloud.push_back(pt_2);

  Eigen::Matrix<float, 4, 4> transform;
  transform << 0.834513, -0.550923, -0.008474, 89571.148438, 0.550986, 0.834372, 0.015428,
    42301.179688, -0.001429, -0.017543, 0.999845, -3.157415, 0.000000, 0.000000, 0.000000, 1.000000;

  pcl::PointCloud<pcl::PointXYZI> cloud_transformed;
  tier4_autoware_utils::transformPointCloud(cloud, cloud_transformed, transform);

  pcl::PointXYZI pt1_gt;
  pt1_gt.x = 89603.187500;
  pt1_gt.y = 42270.878906;
  pt1_gt.z = -13.056946;
  pt1_gt.intensity = 4;

  constexpr float float_error = 0.0001;
  EXPECT_NEAR(cloud_transformed[0].x, pt1_gt.x, float_error);
  EXPECT_NEAR(cloud_transformed[0].y, pt1_gt.y, float_error);
  EXPECT_NEAR(cloud_transformed[0].z, pt1_gt.z, float_error);
  EXPECT_EQ(cloud_transformed[0].intensity, pt1_gt.intensity);
}

TEST(system, empty_point_cloud)
{
  pcl::PointCloud<pcl::PointXYZI> cloud;

  Eigen::Matrix<float, 4, 4> transform;
  transform << 0.834513, -0.550923, -0.008474, 89571.148438, 0.550986, 0.834372, 0.015428,
    42301.179688, -0.001429, -0.017543, 0.999845, -3.157415, 0.000000, 0.000000, 0.000000, 1.000000;

  pcl::PointCloud<pcl::PointXYZI> cloud_transformed;

  EXPECT_NO_THROW(tier4_autoware_utils::transformPointCloud(cloud, cloud_transformed, transform));
  EXPECT_NO_FATAL_FAILURE(
    tier4_autoware_utils::transformPointCloud(cloud, cloud_transformed, transform));
  EXPECT_EQ(cloud_transformed.size(), 0ul);
}
