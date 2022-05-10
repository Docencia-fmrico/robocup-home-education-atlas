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

#include <time.h>

#include "movement/CurrentPosition.h"

#include "behaviortree_cpp_v3/behavior_tree.h"

namespace movement
{

CurrentPosition::CurrentPosition(const std::string& name, const BT::NodeConfiguration& config)
: BT::ActionNodeBase(name, config)
{
}

void
CurrentPosition::halt()
{
  ROS_INFO("CurrentPosition halt");
}

BT::NodeStatus
CurrentPosition::tick()
{
  ROS_INFO("CurrentPosition tick");

  tf::StampedTransform transform;
  try
  {
      //ROS_INFO("Attempting to read pose...");
      listener_.lookupTransform("/map", "/base_footprint", ros::Time(0), transform);

      ROS_INFO("Got a transform! x = %f, y = %f", transform.getOrigin().x(), transform.getRotation().w());
  }
  catch(tf::TransformException& ex)
  {
    ROS_ERROR_STREAM("Transform error of sensor data: " << ex.what() << ", quitting callback");
    return BT::NodeStatus::FAILURE;
  }
  pos_.header.stamp = ros::Time::now();
  pos_.header.frame_id = "base_footprint";
  pos_.pose.position.x = transform.getOrigin().x();
  pos_.pose.position.y = transform.getOrigin().y();
  pos_.pose.position.z = transform.getOrigin().z();
  pos_.pose.orientation.x = transform.getRotation().x();
  pos_.pose.orientation.y = transform.getRotation().y();
  pos_.pose.orientation.z = transform.getRotation().z();
  pos_.pose.orientation.w = transform.getRotation().w();

  ROS_INFO("Current position: x = %f, y = %f", pos_.pose.position.x, pos_.pose.orientation.y);

  setOutput<geometry_msgs::PoseStamped>("current_pos", pos_);

  return BT::NodeStatus::SUCCESS;
}

}  // namespace movement

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<movement::CurrentPosition>("CurrentPosition");
}