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

#include "find_my_mates/ApproachOperator.h"

#include "behaviortree_cpp_v3/behavior_tree.h"

#include "ros/ros.h"

namespace find_my_mates
{

ApproachOperator::ApproachOperator(
  const std::string& name,
  const std::string& action_name,
  const BT::NodeConfiguration& config)
: BTNavAction(name, action_name, config)
{
}

void
ApproachOperator::on_halt()
{
  ROS_INFO("ApproachOperator halt");
}

void
ApproachOperator::on_start()
{
  move_base_msgs::MoveBaseGoal goal;
  auto res = getInput<geometry_msgs::Pose>("position");

  if( !res )
  {
      ROS_INFO("ERROR");
  }
  
  geometry_msgs::Pose pos = getInput<geometry_msgs::Pose>("position").value();

  ROS_INFO("Got position! x = %f, y = %f", pos.position.x, pos.orientation.w);
  
  goal.target_pose.header.frame_id = "map";
  goal.target_pose.header.stamp = ros::Time::now();
  goal.target_pose.pose.position.x = pos.position.x;
  goal.target_pose.pose.position.y = pos.position.y;
  goal.target_pose.pose.position.z = pos.position.z;
  goal.target_pose.pose.orientation.x = pos.orientation.x;
  goal.target_pose.pose.orientation.y = pos.orientation.y;
  goal.target_pose.pose.orientation.z = pos.orientation.z;
  goal.target_pose.pose.orientation.w = pos.orientation.w;

  set_goal(goal);

  ROS_INFO("ApproachOperator start");
}

BT::NodeStatus
ApproachOperator::on_tick()
{
  ROS_INFO("ApproachOperator tick");

  return BT::NodeStatus::RUNNING;
}

void
ApproachOperator::on_feedback(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback)
{
	ROS_INFO("Current position %lf", feedback->base_position.pose.position.x);
}

}  // namespace find_my_mates

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<find_my_mates::ApproachOperator>(
        name, "move_base", config);
    };

  factory.registerBuilder<find_my_mates::ApproachOperator>(
    "ApproachOperator", builder);
}