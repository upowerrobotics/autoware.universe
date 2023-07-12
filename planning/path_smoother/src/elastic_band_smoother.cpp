// Copyright 2023 TIER IV, Inc.
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

#include "path_smoother/elastic_band_smoother.hpp"

#include "interpolation/spline_interpolation_points_2d.hpp"
#include "path_smoother/utils/geometry_utils.hpp"
#include "path_smoother/utils/trajectory_utils.hpp"
#include "rclcpp/time.hpp"

#include <chrono>
#include <limits>

namespace path_smoother
{
namespace
{
template <class T>
std::vector<T> concatVectors(const std::vector<T> & prev_vector, const std::vector<T> & next_vector)
{
  std::vector<T> concatenated_vector;
  concatenated_vector.insert(concatenated_vector.end(), prev_vector.begin(), prev_vector.end());
  concatenated_vector.insert(concatenated_vector.end(), next_vector.begin(), next_vector.end());
  return concatenated_vector;
}

StringStamped createStringStamped(const rclcpp::Time & now, const std::string & data)
{
  StringStamped msg;
  msg.stamp = now;
  msg.data = data;
  return msg;
}

void setZeroVelocityAfterStopPoint(std::vector<TrajectoryPoint> & traj_points)
{
  const auto opt_zero_vel_idx = motion_utils::searchZeroVelocityIndex(traj_points);
  if (opt_zero_vel_idx) {
    for (size_t i = opt_zero_vel_idx.get(); i < traj_points.size(); ++i) {
      traj_points.at(i).longitudinal_velocity_mps = 0.0;
    }
  }
}

bool hasZeroVelocity(const TrajectoryPoint & traj_point)
{
  constexpr double zero_vel = 0.0001;
  return std::abs(traj_point.longitudinal_velocity_mps) < zero_vel;
}
}  // namespace

ElasticBandSmoother::ElasticBandSmoother(const rclcpp::NodeOptions & node_options)
: Node("path_smoother", node_options), time_keeper_ptr_(std::make_shared<TimeKeeper>())
{
  // interface publisher
  traj_pub_ = create_publisher<Trajectory>("~/output/traj", 1);
  path_pub_ = create_publisher<Path>("~/output/path", 1);

  // interface subscriber
  path_sub_ = create_subscription<Path>(
    "~/input/path", 1, std::bind(&ElasticBandSmoother::onPath, this, std::placeholders::_1));
  odom_sub_ = create_subscription<Odometry>(
    "~/input/odometry", 1, [this](const Odometry::SharedPtr msg) { ego_state_ptr_ = msg; });

  // debug publisher
  debug_extended_traj_pub_ = create_publisher<Trajectory>("~/debug/extended_traj", 1);
  debug_calculation_time_pub_ = create_publisher<StringStamped>("~/debug/calculation_time", 1);

  {  // parameters
    // parameters for ego nearest search
    ego_nearest_param_ = EgoNearestParam(this);

    // parameters for trajectory
    common_param_ = CommonParam(this);
  }

  eb_path_smoother_ptr_ = std::make_shared<EBPathSmoother>(
    this, enable_debug_info_, ego_nearest_param_, common_param_, time_keeper_ptr_);

  // reset planners
  initializePlanning();

  // set parameter callback
  set_param_res_ = this->add_on_set_parameters_callback(
    std::bind(&ElasticBandSmoother::onParam, this, std::placeholders::_1));
}

rcl_interfaces::msg::SetParametersResult ElasticBandSmoother::onParam(
  const std::vector<rclcpp::Parameter> & parameters)
{
  using tier4_autoware_utils::updateParam;

  // parameters for ego nearest search
  ego_nearest_param_.onParam(parameters);

  // parameters for trajectory
  common_param_.onParam(parameters);

  // parameters for core algorithms
  eb_path_smoother_ptr_->onParam(parameters);

  // reset planners
  initializePlanning();

  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";
  return result;
}

void ElasticBandSmoother::initializePlanning()
{
  RCLCPP_INFO(get_logger(), "Initialize planning");

  eb_path_smoother_ptr_->initialize(false, common_param_);
  resetPreviousData();
}

void ElasticBandSmoother::resetPreviousData()
{
  eb_path_smoother_ptr_->resetPreviousData();

  prev_optimized_traj_points_ptr_ = nullptr;
}

void ElasticBandSmoother::onPath(const Path::SharedPtr path_ptr)
{
  time_keeper_ptr_->init();
  time_keeper_ptr_->tic(__func__);

  // check if data is ready and valid
  if (!isDataReady(*path_ptr, *get_clock())) {
    return;
  }

  // 0. return if path is backward
  // TODO(murooka): support backward path
  const auto is_driving_forward = driving_direction_checker_.isDrivingForward(path_ptr->points);
  if (!is_driving_forward) {
    RCLCPP_WARN_THROTTLE(
      get_logger(), *get_clock(), 5000,
      "Backward path is NOT supported. Just converting path to trajectory");

    const auto traj_points = trajectory_utils::convertToTrajectoryPoints(path_ptr->points);
    const auto output_traj_msg = trajectory_utils::createTrajectory(path_ptr->header, traj_points);
    traj_pub_->publish(output_traj_msg);
    path_pub_->publish(*path_ptr);
    return;
  }

  // 1. create planner data
  const auto planner_data = createPlannerData(*path_ptr);

  // 2. generate optimized trajectory
  const auto optimized_traj_points = generateOptimizedTrajectory(planner_data);

  // 3. extend trajectory to connect the optimized trajectory and the following path smoothly
  auto full_traj_points = extendTrajectory(planner_data.traj_points, optimized_traj_points);

  // 4. set zero velocity after stop point
  setZeroVelocityAfterStopPoint(full_traj_points);

  time_keeper_ptr_->toc(__func__, "");
  *time_keeper_ptr_ << "========================================";
  time_keeper_ptr_->endLine();

  // publish calculation_time
  // NOTE: This function must be called after measuring onPath calculation time
  const auto calculation_time_msg = createStringStamped(now(), time_keeper_ptr_->getLog());
  debug_calculation_time_pub_->publish(calculation_time_msg);

  const auto output_traj_msg =
    trajectory_utils::createTrajectory(path_ptr->header, full_traj_points);
  traj_pub_->publish(output_traj_msg);
  const auto output_path_msg = trajectory_utils::create_path(*path_ptr, full_traj_points);
  path_pub_->publish(output_path_msg);
}

bool ElasticBandSmoother::isDataReady(const Path & path, rclcpp::Clock clock) const
{
  if (!ego_state_ptr_) {
    RCLCPP_INFO_SKIPFIRST_THROTTLE(get_logger(), clock, 5000, "Waiting for ego pose and twist.");
    return false;
  }

  if (path.points.size() < 2) {
    RCLCPP_INFO_SKIPFIRST_THROTTLE(get_logger(), clock, 5000, "Path points size is less than 1.");
    return false;
  }

  if (path.left_bound.empty() || path.right_bound.empty()) {
    RCLCPP_INFO_SKIPFIRST_THROTTLE(
      get_logger(), clock, 5000, "Left or right bound in path is empty.");
    return false;
  }

  return true;
}

PlannerData ElasticBandSmoother::createPlannerData(const Path & path) const
{
  // create planner data
  PlannerData planner_data;
  planner_data.header = path.header;
  planner_data.traj_points = trajectory_utils::convertToTrajectoryPoints(path.points);
  planner_data.left_bound = path.left_bound;
  planner_data.right_bound = path.right_bound;
  planner_data.ego_pose = ego_state_ptr_->pose.pose;
  planner_data.ego_vel = ego_state_ptr_->twist.twist.linear.x;
  return planner_data;
}

std::vector<TrajectoryPoint> ElasticBandSmoother::generateOptimizedTrajectory(
  const PlannerData & planner_data)
{
  time_keeper_ptr_->tic(__func__);

  const auto & input_traj_points = planner_data.traj_points;

  // 1. calculate trajectory with Elastic Band
  auto optimized_traj_points = optimizeTrajectory(planner_data);

  // 2. update velocity
  applyInputVelocity(optimized_traj_points, input_traj_points, planner_data.ego_pose);

  time_keeper_ptr_->toc(__func__, " ");
  return optimized_traj_points;
}

std::vector<TrajectoryPoint> ElasticBandSmoother::optimizeTrajectory(
  const PlannerData & planner_data)
{
  time_keeper_ptr_->tic(__func__);
  const auto & p = planner_data;

  const auto eb_traj = eb_path_smoother_ptr_->getEBTrajectory(planner_data);
  if (!eb_traj) return getPrevOptimizedTrajectory(p.traj_points);

  time_keeper_ptr_->toc(__func__, "    ");
  return *eb_traj;
}

std::vector<TrajectoryPoint> ElasticBandSmoother::getPrevOptimizedTrajectory(
  const std::vector<TrajectoryPoint> & traj_points) const
{
  if (prev_optimized_traj_points_ptr_) return *prev_optimized_traj_points_ptr_;
  return traj_points;
}

void ElasticBandSmoother::applyInputVelocity(
  std::vector<TrajectoryPoint> & output_traj_points,
  const std::vector<TrajectoryPoint> & input_traj_points,
  const geometry_msgs::msg::Pose & ego_pose) const
{
  time_keeper_ptr_->tic(__func__);

  // crop forward for faster calculation
  const double output_traj_length = motion_utils::calcArcLength(output_traj_points);
  constexpr double margin_traj_length = 10.0;
  const auto forward_cropped_input_traj_points = [&]() {
    const size_t ego_seg_idx =
      trajectory_utils::findEgoSegmentIndex(input_traj_points, ego_pose, ego_nearest_param_);
    return motion_utils::cropForwardPoints(
      input_traj_points, ego_pose.position, ego_seg_idx, output_traj_length + margin_traj_length);
  }();

  // update velocity
  size_t input_traj_start_idx = 0;
  for (size_t i = 0; i < output_traj_points.size(); i++) {
    // crop backward for efficient calculation
    const auto cropped_input_traj_points = std::vector<TrajectoryPoint>{
      forward_cropped_input_traj_points.begin() + input_traj_start_idx,
      forward_cropped_input_traj_points.end()};

    const size_t nearest_seg_idx = trajectory_utils::findEgoSegmentIndex(
      cropped_input_traj_points, output_traj_points.at(i).pose, ego_nearest_param_);
    input_traj_start_idx = nearest_seg_idx;

    // calculate velocity with zero order hold
    const double velocity = cropped_input_traj_points.at(nearest_seg_idx).longitudinal_velocity_mps;
    output_traj_points.at(i).longitudinal_velocity_mps = velocity;
  }

  // insert stop point explicitly
  const auto stop_idx = motion_utils::searchZeroVelocityIndex(forward_cropped_input_traj_points);
  if (stop_idx) {
    const auto input_stop_pose = forward_cropped_input_traj_points.at(stop_idx.get()).pose;
    const size_t stop_seg_idx = trajectory_utils::findEgoSegmentIndex(
      output_traj_points, input_stop_pose, ego_nearest_param_);

    // calculate and insert stop pose on output trajectory
    trajectory_utils::insertStopPoint(output_traj_points, input_stop_pose, stop_seg_idx);
  }

  time_keeper_ptr_->toc(__func__, "    ");
}

std::vector<TrajectoryPoint> ElasticBandSmoother::extendTrajectory(
  const std::vector<TrajectoryPoint> & traj_points,
  const std::vector<TrajectoryPoint> & optimized_traj_points) const
{
  time_keeper_ptr_->tic(__func__);

  const auto & joint_start_pose = optimized_traj_points.back().pose;

  // calculate end idx of optimized points on path points
  const size_t joint_start_traj_seg_idx =
    trajectory_utils::findEgoSegmentIndex(traj_points, joint_start_pose, ego_nearest_param_);

  // crop trajectory for extension
  constexpr double joint_traj_max_length_for_smoothing = 15.0;
  constexpr double joint_traj_min_length_for_smoothing = 5.0;
  const auto joint_end_traj_point_idx = trajectory_utils::getPointIndexAfter(
    traj_points, joint_start_pose.position, joint_start_traj_seg_idx,
    joint_traj_max_length_for_smoothing, joint_traj_min_length_for_smoothing);

  // calculate full trajectory points
  const auto full_traj_points = [&]() {
    if (!joint_end_traj_point_idx) {
      return optimized_traj_points;
    }

    const auto extended_traj_points = std::vector<TrajectoryPoint>{
      traj_points.begin() + *joint_end_traj_point_idx, traj_points.end()};

    // NOTE: if optimized_traj_points's back is non zero velocity and extended_traj_points' front is
    // zero velocity, the zero velocity will be inserted in the whole joint trajectory.
    auto modified_optimized_traj_points = optimized_traj_points;
    if (!extended_traj_points.empty() && !modified_optimized_traj_points.empty()) {
      modified_optimized_traj_points.back().longitudinal_velocity_mps =
        extended_traj_points.front().longitudinal_velocity_mps;
    }

    return concatVectors(modified_optimized_traj_points, extended_traj_points);
  }();

  // resample trajectory points
  auto resampled_traj_points = trajectory_utils::resampleTrajectoryPoints(
    full_traj_points, common_param_.output_delta_arc_length);

  // update stop velocity on joint
  for (size_t i = joint_start_traj_seg_idx + 1; i <= joint_end_traj_point_idx; ++i) {
    if (hasZeroVelocity(traj_points.at(i))) {
      if (i != 0 && !hasZeroVelocity(traj_points.at(i - 1))) {
        // Here is when current point is 0 velocity, but previous point is not 0 velocity.
        const auto & input_stop_pose = traj_points.at(i).pose;
        const size_t stop_seg_idx = trajectory_utils::findEgoSegmentIndex(
          resampled_traj_points, input_stop_pose, ego_nearest_param_);

        // calculate and insert stop pose on output trajectory
        trajectory_utils::insertStopPoint(resampled_traj_points, input_stop_pose, stop_seg_idx);
      }
    }
  }

  time_keeper_ptr_->toc(__func__, "  ");
  return resampled_traj_points;
}
}  // namespace path_smoother

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(path_smoother::ElasticBandSmoother)
