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

#include "find_my_mates/GetPosition.h"

#include "behaviortree_cpp_v3/behavior_tree.h"

#include "ros/ros.h"

namespace find_my_mates
{

GetPosition::GetPosition(const std::string& name, const BT::NodeConfiguration & config)
: BT::ActionNodeBase(name, config)
{
}

void
GetPosition::halt()
{
  ROS_INFO("GetPosition halt");
}

BT::NodeStatus
GetPosition::tick()
{
  ROS_INFO("GetPosition tick");

  tf::StampedTransform transform;
  try
  {
      //ROS_INFO("Attempting to read pose...");
      listener_.lookupTransform("/map", "/base_link", ros::Time(0), transform);

      ROS_INFO("Got a transform! x = %f, y = %f", transform.getOrigin().x(), transform.getRotation().w());
  }
  catch(tf::TransformException& ex)
  {
    ROS_ERROR_STREAM("Transform error of sensor data: " << ex.what() << ", quitting callback");
    return BT::NodeStatus::FAILURE;
  }
  
  pos_.position.x = transform.getOrigin().x();
  pos_.position.y = transform.getOrigin().y();
  pos_.position.z = transform.getOrigin().z();
  pos_.orientation.x = transform.getRotation().x();
  pos_.orientation.y = transform.getRotation().y();
  pos_.orientation.z = transform.getRotation().z();
  pos_.orientation.w = transform.getRotation().w();

  ROS_INFO("Got a position! x = %f, y = %f", pos_.position.x, pos_.orientation.w);

  setOutput<geometry_msgs::Pose>("position", pos_);

  return BT::NodeStatus::SUCCESS;
}

}  // namespace find_my_mates

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<find_my_mates::GetPosition>("GetPosition");
}
