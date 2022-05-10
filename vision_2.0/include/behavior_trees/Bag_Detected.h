
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


#ifndef FSM_ROBOCUP_BAG_DETECTED_H
#define FSM_ROBOCUP_BAG_DETECTED_H

#include <string>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "geometry_msgs/Twist.h"

#include "fsm_robocup/bbx_info.h"

#include "ros/ros.h"

namespace fsm_robocup
{

class Bag_Detected : public BT::ConditionNode
{
  public:
    ros::NodeHandle n_;
    ros::Time last_lecture;
    explicit Bag_Detected(const std::string& name);
    void messageCallback(const fsm_robocup::bbx_info::ConstPtr& msg);
    BT::NodeStatus tick();
    float dist;
  private:
    ros::Subscriber sub_;
};

}  // namespace fsm_robocup

#endif  // FSM_ROBOCUP_BAG_DETECTED_H
