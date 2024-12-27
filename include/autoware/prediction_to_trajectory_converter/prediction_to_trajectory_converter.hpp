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

#ifndef AUTOWARE__PLANNING_TOPIC_CONVERTER__PATH_TO_TRAJECTORY_HPP_
#define AUTOWARE__PLANNING_TOPIC_CONVERTER__PATH_TO_TRAJECTORY_HPP_

#include "autoware/planning_topic_converter/converter_base.hpp"
#include "rclcpp/rclcpp.hpp"

#include <autoware/universe_utils/ros/uuid_helper.hpp>

#include "autoware_new_planning_msgs/msg/trajectories.hpp"
#include <autoware_new_planning_msgs/msg/detail/trajectory_generator_info__struct.hpp>
#include <autoware_perception_msgs/msg/detail/predicted_objects__struct.hpp>
#include <autoware_perception_msgs/msg/predicted_objects.hpp>
#include <autoware_planning_msgs/msg/trajectory.hpp>
#include <unique_identifier_msgs/msg/detail/uuid__struct.hpp>
#include <unique_identifier_msgs/msg/uuid.hpp>

#include <string>

namespace autoware::prediction_to_trajectory_converter
{

using autoware::planning_topic_converter::ConverterBase;
using autoware_new_planning_msgs::msg::Trajectories;
using autoware_new_planning_msgs::msg::Trajectory;
using autoware_new_planning_msgs::msg::TrajectoryGeneratorInfo;
using autoware_perception_msgs::msg::PredictedObjects;
using autoware_planning_msgs::msg::TrajectoryPoint;
using unique_identifier_msgs::msg::UUID;

class PredictionToTrajectory : public ConverterBase<PredictedObjects, Trajectories>
{
public:
  explicit PredictionToTrajectory(const rclcpp::NodeOptions & options);

private:
  void process(const PredictedObjects::ConstSharedPtr msg) override;
  UUID generator_uuid_;
};

}  // namespace autoware::prediction_to_trajectory_converter

#endif  // AUTOWARE__PLANNING_TOPIC_CONVERTER__PATH_TO_TRAJECTORY_HPP_
