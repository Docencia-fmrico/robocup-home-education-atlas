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

#include "movement/ApproachPerson.h"

#include "behaviortree_cpp_v3/behavior_tree.h"

#include "geometry_msgs/Twist.h"

namespace movement
{

ApproachPerson::ApproachPerson(const std::string& name)
: BT::ActionNodeBase(name, {}), velocity_pid(0.0, 5.0, 0.0, 0.2), turn_pid(0.0, 3.2, 0.0, 0.5)
{
  pub_vel_ = n_.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity",1);
  sub_ = n_.subscribe("bbx_custom_topic", 1, &movement::ApproachPerson::messageCallback, this);
}

void
ApproachPerson::messageCallback(const vision::bbx_info::ConstPtr& msg){
  dist = msg->dist;
  px = msg->px;
  
}

void
ApproachPerson::halt()
{
  ROS_INFO("ApproachPerson halt");
}

BT::NodeStatus
ApproachPerson::tick()
{ 
  ROS_INFO("ApproachPerson tick");
  
  if(dist < 0.5*0.001)
  {
    return BT::NodeStatus::SUCCESS;
  }
  
  geometry_msgs::Twist vel_msgs;
  double speed_clamped = std::clamp(dist-0.8, -5.0, 5.0);
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
  factory.registerNodeType<movement::ApproachPerson>("ApproachPerson");
}
