
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

#include "behavior_trees/Is_bumped.h"

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

#include "geometry_msgs/Twist.h"
#include "ros/ros.h"

namespace fsm_robocup
{

Is_bumped::Is_bumped(const std::string& name)
: BT::ConditionNode(name, {}), pressed_(false)
{
  sub_bumper_ = n_.subscribe("/mobile_base/events/bumper", 1, &Is_bumped::bumperCallback, this);
}

void
Is_bumped::bumperCallback(const kobuki_msgs::BumperEvent::ConstPtr& msg)
{
  if(pressed_ == false)
    pressed_ = msg->state == kobuki_msgs::BumperEvent::PRESSED;
}

BT::NodeStatus
Is_bumped::tick()
{
  std::cout << "Nodo Is bumped" << std::endl;
  if (pressed_)
      {
        press_ts_ = ros::Time::now();
        setOutput<ros::Time>("object1",press_ts_);
        std::cout << "Pressed" << std::endl;
        return BT::NodeStatus::SUCCESS;
      }
    else  
    {
      std::cout << "not" << std::endl;
      return BT::NodeStatus::FAILURE;
    }
}

}  // namespace fsm_robocup

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<fsm_robocup::Is_bumped>("Is_bumped");
}
