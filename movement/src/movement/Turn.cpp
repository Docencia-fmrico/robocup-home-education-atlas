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

#include "movement/Turn.h"

#include "behaviortree_cpp_v3/behavior_tree.h"

#include "geometry_msgs/Twist.h"

#include "ros/ros.h"

namespace movement
{

Turn::Turn(const std::string& name)
: BT::ActionNodeBase(name, {}), n_()
{
  pub_vel_ = n_.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity",1);
}

void
Turn::halt()
{
  ROS_INFO("Turn halt");
}

BT::NodeStatus
Turn::tick()
{
  ROS_INFO("Turn tick");
  std::string error;
  geometry_msgs::Twist cmd;
  cmd.angular.z = 0.5f;
  pub_vel_.publish(cmd);
  return BT::NodeStatus::RUNNING;
  
}

}  // namespace movement

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<movement::Turn>("Turn");
}
