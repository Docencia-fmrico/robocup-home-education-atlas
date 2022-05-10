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

#ifndef MOVEMENT_APPROACHPERSON_H
#define MOVEMENT_APPROACHPERSON_H

#include <algorithm>
#include <string>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

#include <tf/transform_listener.h>
#include <geometry_msgs/Pose.h>

#include "PIDController.h"
#include "vision/bbx_info.h"

#include "ros/ros.h"

namespace movement
{

class ApproachPerson : public BT::ActionNodeBase
{
  public:
    ros::NodeHandle n_;
    double dist;
    int px;
    explicit ApproachPerson(const std::string& name);
    void messageCallback(const vision::bbx_info::ConstPtr& msg);
    void halt();
    BT::NodeStatus tick();

  private:
    static constexpr float GOING_FORWARD_VEL = 0.3;
    ros::Publisher pub_vel_;
    ros::Subscriber sub_;
    br2_tracking::PIDController velocity_pid;
    br2_tracking::PIDController turn_pid;
  };


}  // namespace movement

#endif  // MOVEMENT_APPROACHPERSON_H
