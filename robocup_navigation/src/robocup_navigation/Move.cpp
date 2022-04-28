// Copyright 2022 Team Atlas
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

#include <string>

#include "robocup_navigation/Move.h"
#include "robocup_navigation/Goal.h"

#include "behaviortree_cpp_v3/behavior_tree.h"

#include "ros/ros.h"

#include <stdlib.h>

namespace robocup_navigation
{

Move::Move(
  const std::string& name,
  const std::string& action_name,
  const BT::NodeConfiguration& config)
: BTNavAction(name, action_name, config), new_goal_(false)
{
}

void
Move::on_halt()
{
  ROS_INFO("Move halt");
}

void
Move::on_start()
{
  move_base_msgs::MoveBaseGoal goal;

  pos_ = getInput<geometry_msgs::Pose>("position").value();

  ROS_INFO("Got position! x = %f, y = %f", pos_.position.x, pos_.orientation.w);

  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();
  goal.target_pose.pose.position.x = pos_.position.x;
  goal.target_pose.pose.position.y = pos_.position.y;
  goal.target_pose.pose.position.z = pos_.position.z;
  goal.target_pose.pose.orientation.x = pos_.orientation.x;
  goal.target_pose.pose.orientation.y = pos_.orientation.y;
  goal.target_pose.pose.orientation.z = pos_.orientation.z;
  goal.target_pose.pose.orientation.w = pos_.orientation.w;

  set_goal(goal);

  ROS_INFO("Move start");
}

BT::NodeStatus
Move::on_tick()
{
  ROS_INFO("Move tick");

  return BT::NodeStatus::RUNNING;
}

void
Move::on_feedback(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback)
{
	ROS_INFO("Current position %lf %lf", feedback->base_position.pose.orientation.z, feedback->base_position.pose.orientation.w);
}

}  // namespace robocup_navigation

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<robocup_navigation::Move>(
        name, "move_base", config);
    };

  factory.registerBuilder<robocup_navigation::Move>(
    "Move", builder);
}