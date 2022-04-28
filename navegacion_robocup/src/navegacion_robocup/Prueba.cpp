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

#include "navegacion_robocup/Prueba.h"
#include "geometry_msgs/PoseStamped.h"

namespace navegacion_robocup
{

Prueba::Prueba(const std::string& name, const BT::NodeConfiguration & config)
: BT::ActionNodeBase(name, config), n_()
{  
  ROS_INFO("Prueba inicio");
}

void 
Prueba::callback()
{
  ROS_INFO("Prueba callback");
}

void
Prueba::halt()
{
  ROS_INFO("Prueba halt");
}

BT::NodeStatus
Prueba::tick()
{
 /* ros::Publisher pub = n_.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal",10);

  geometry_msgs::PoseStamped msg;
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = "map";

  msg.pose.position.x = 5.651284694671631;
  msg.pose.position.y = 3.162477970123291;
  msg.pose.position.z = -0.0006666183471679688;
  msg.pose.orientation.x = 0.0;
  msg.pose.orientation.y = 0.0;
  msg.pose.orientation.z = 0.0;
  msg.pose.orientation.w = 1.0;
  
  ROS_INFO("Prueba tick");
  pub.publish(msg);*/
  return BT::NodeStatus::RUNNING;
}

}  // namespace navegacion_robocup

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<navegacion_robocup::Prueba>("Prueba");
}