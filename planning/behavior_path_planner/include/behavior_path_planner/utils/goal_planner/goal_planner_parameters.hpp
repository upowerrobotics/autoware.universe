
// Copyright 2022 TIER IV, Inc.
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

#ifndef BEHAVIOR_PATH_PLANNER__UTILS__GOAL_PLANNER__GOAL_PLANNER_PARAMETERS_HPP_
#define BEHAVIOR_PATH_PLANNER__UTILS__GOAL_PLANNER__GOAL_PLANNER_PARAMETERS_HPP_

#include "behavior_path_planner/utils/geometric_parallel_parking/geometric_parallel_parking.hpp"

#include <freespace_planning_algorithms/abstract_algorithm.hpp>
#include <freespace_planning_algorithms/astar_search.hpp>
#include <freespace_planning_algorithms/rrtstar.hpp>

#include <string>
#include <vector>

namespace behavior_path_planner
{

using freespace_planning_algorithms::AstarParam;
using freespace_planning_algorithms::PlannerCommonParam;
using freespace_planning_algorithms::RRTStarParam;

enum class ParkingPolicy {
  LEFT_SIDE = 0,
  RIGHT_SIDE,
};

struct GoalPlannerParameters
{
  // general  params
  double minimum_request_length;
  double th_arrived_distance;
  double th_stopped_velocity;
  double th_stopped_time;
  double th_blinker_on_lateral_offset;

  // goal search
  std::string search_priority;   // "efficient_path" or "close_goal"
  ParkingPolicy parking_policy;  // "left_side" or "right_side"
  double forward_goal_search_length;
  double backward_goal_search_length;
  double goal_search_interval;
  double longitudinal_margin;
  double max_lateral_offset;
  double lateral_offset_interval;
  double ignore_distance_from_lane_start;
  double margin_from_boundary;

  // occupancy grid map
  bool use_occupancy_grid;
  bool use_occupancy_grid_for_longitudinal_margin;
  double occupancy_grid_collision_check_margin;
  int theta_size;
  int obstacle_threshold;

  // object recognition
  bool use_object_recognition;
  double object_recognition_collision_check_margin;

  // pull over general params
  double pull_over_velocity;
  double pull_over_minimum_velocity;
  double decide_path_distance;
  double maximum_deceleration;
  double maximum_jerk;

  // shift path
  bool enable_shift_parking;
  int shift_sampling_num;
  double maximum_lateral_jerk;
  double minimum_lateral_jerk;
  double deceleration_interval;
  double after_shift_straight_distance;

  // parallel parking
  bool enable_arc_forward_parking;
  bool enable_arc_backward_parking;
  ParallelParkingParameters parallel_parking_parameters;

  // freespace parking
  bool enable_freespace_parking;
  std::string freespace_parking_algorithm;
  double freespace_parking_velocity;
  double vehicle_shape_margin;
  PlannerCommonParam freespace_parking_common_parameters;
  AstarParam astar_parameters;
  RRTStarParam rrt_star_parameters;

  // debug
  bool print_debug_info;
};
}  // namespace behavior_path_planner

#endif  // BEHAVIOR_PATH_PLANNER__UTILS__GOAL_PLANNER__GOAL_PLANNER_PARAMETERS_HPP_
