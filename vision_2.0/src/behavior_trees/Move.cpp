
// Copyright 2019 Intelligent Robotics Lab
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

#include "behavior_trees/Move.h"

#include "behaviortree_cpp_v3/behavior_tree.h"

#include "ros/ros.h"

namespace fsm_robocup
{

Move::Move(
  const std::string& name,
  const std::string & action_name,
  const BT::NodeConfiguration & config)
: BTNavAction(name, action_name, config), counter_(0), go_home(false)
{
}

void
Move::on_halt()
{
  ROS_INFO("Move halt");
}

BT::NodeStatus
Move::on_tick()
{
  if(counter_ == 0){
    std::cout << "New Goal===========================" << std::endl;
    move_base_msgs::MoveBaseGoal goal;

    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose.position.x = 0.0;
    goal.target_pose.pose.position.y = 0.0;
    goal.target_pose.pose.position.z = 0.0;
    goal.target_pose.pose.orientation.x = 0.0;
    goal.target_pose.pose.orientation.y = 0.0;
    goal.target_pose.pose.orientation.z = 0.0;
    goal.target_pose.pose.orientation.w = 1.0;

    set_goal(goal);
    counter_++;
  }
  return BT::NodeStatus::RUNNING;
}

void
Move::on_feedback(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback)
{
	ROS_INFO("Current count %lf", feedback->base_position.pose.position.x);
}

}  // namespace fsm_robocup

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
    BT::NodeBuilder builder =
    [](const std::string & name, const BT::NodeConfiguration & config)
    {
      return std::make_unique<fsm_robocup::Move>(
        name, "move_base", config);
    };

  factory.registerBuilder<fsm_robocup::Move>(
    "Move", builder);
}
