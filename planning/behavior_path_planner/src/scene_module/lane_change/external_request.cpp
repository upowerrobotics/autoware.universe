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

#include "behavior_path_planner/scene_module/lane_change/external_request.hpp"

#include "behavior_path_planner/scene_module/scene_module_visitor.hpp"
#include "behavior_path_planner/utils/lane_change/utils.hpp"
#include "behavior_path_planner/utils/path_utils.hpp"

#include <lanelet2_extension/utility/utilities.hpp>

#include <algorithm>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace behavior_path_planner
{
ExternalRequestLaneChange::ExternalRequestLaneChange(
  const std::shared_ptr<LaneChangeParameters> & parameters, Direction direction)
: NormalLaneChange(parameters, direction)
{
}

lanelet::ConstLanelets ExternalRequestLaneChange::getLaneChangeLanes(
  const lanelet::ConstLanelets & current_lanes) const
{
  if (current_lanes.empty()) {
    return {};
  }
  // Get lane change lanes
  lanelet::ConstLanelet current_lane;
  lanelet::utils::query::getClosestLanelet(current_lanes, getEgoPose(), &current_lane);

  const auto minimum_lane_changing_length = planner_data_->parameters.minimum_lane_changing_length;

  const auto lane_change_prepare_length =
    std::max(getEgoVelocity() * parameters_->prepare_duration, minimum_lane_changing_length);

  const auto & route_handler = getRouteHandler();

  const auto current_check_lanes =
    route_handler->getLaneletSequence(current_lane, getEgoPose(), 0.0, lane_change_prepare_length);

  const auto lane_change_lane =
    route_handler->getLaneChangeTargetExceptPreferredLane(current_check_lanes, direction_);

  if (lane_change_lane) {
    return route_handler->getLaneletSequence(
      lane_change_lane.get(), getEgoPose(), lane_change_lane_length_, lane_change_lane_length_);
  }

  return {};
}
}  // namespace behavior_path_planner
