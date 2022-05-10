
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

#ifndef FSM_ROBOCUP_FOLLOW_PERSON_H
#define FSM_ROBOCUP_FOLLOW_PERSON_H

#include <algorithm>
#include <string>
#include <ros/ros.h>

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"
#include "geometry_msgs/Twist.h"
#include "PIDController.hpp"
#include "fsm_robocup/bbx_info.h"
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>


namespace fsm_robocup
{

class Follow_Person : public BT::ActionNodeBase
{
  public:
    ros::NodeHandle n_;
    double dist;
    int px, xmax, xmin;
    explicit Follow_Person(const std::string& name);
    void messageCallback(const fsm_robocup::bbx_info::ConstPtr& msg);
    void halt();
    BT::NodeStatus tick();

  private:
    static constexpr float GOING_FORWARD_VEL = 0.3;
    ros::Publisher pub_vel_;
    ros::Subscriber sub_;
    br2_tracking::PIDController velocity_pid;
    br2_tracking::PIDController turn_pid;
};

}  // namespace fsm_robocup

#endif  // FSM_ROBOCUP_FOLLOW_PERSON_H
