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

#include "movement/Follow_Person.h"

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

#include "geometry_msgs/Twist.h"

#include "ros/ros.h"
#include "movement/bbx_info.h"

namespace movement
{

Follow_Person::Follow_Person(const std::string& name)
: BT::ActionNodeBase(name, {}), velocity_pid(0.0, 5.0, 0.0, 0.2), turn_pid(0.0, 3.2, 0.0, 0.5)
{
  pub_vel_ = n_.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity",1);
  sub_ = n_.subscribe("bbx_person", 1, &movement::Follow_Person::messageCallback, this);
}

void
Follow_Person::messageCallback(const movement::bbx_info::ConstPtr& msg){
  dist = msg->dist;
  px = msg->px;
  ROS_INFO("distancia %f.",dist);
}

void
Follow_Person::halt()
{
  ROS_INFO("finished going forward");
}

BT::NodeStatus
Follow_Person::tick()
{
  ROS_INFO("siguiendo persona");
  geometry_msgs::Twist vel_msgs;
  double speed_clamped = std::clamp(dist -1, -5.0, 5.0);
  double angle_clamped = (px-320)/(-100);

  float speed_pid = velocity_pid.get_output(speed_clamped)*1.0f;
  float angle_pid = turn_pid.get_output(angle_clamped)*1.0f;

  vel_msgs.angular.z = angle_pid;
  vel_msgs.linear.x = speed_pid;
  ROS_INFO("angle_pid = %f angle clamped = %lf",angle_pid, angle_clamped);
  ROS_INFO("speed_pid = %f speed clamped = %lf",speed_pid,speed_clamped);
      
  pub_vel_.publish(vel_msgs);
  
  return BT::NodeStatus::RUNNING;
}

}  // namespace movement

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<movement::Follow_Person>("Follow_Person");
}