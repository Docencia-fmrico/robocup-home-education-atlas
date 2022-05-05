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

#include "vision/DetectOperator.h"

#include "behaviortree_cpp_v3/behavior_tree.h"

#include "nav_msgs/GetPlan.h"
#include "nav_msgs/Path.h"

#include <string>

namespace vision
{

DetectOperator::DetectOperator(const std::string& name, const BT::NodeConfiguration& config)
: BT::ConditionNode(name, config), n_(), buffer_(), listener_(buffer_)
{
  plan_client_ = n_.serviceClient<nav_msgs::GetPlan>("move_base/make_plan");
}


BT::NodeStatus
DetectOperator::tick()
{
  ROS_INFO("DetectOperator tick");

  std::string error;

  if(buffer_.canTransform("map", "operator/0", ros::Time(0), ros::Duration(1.0), &error))
  {
    map2op_msg_ = buffer_.lookupTransform("map", "operator/0", ros::Time(0));
    
    tf2::fromMsg(map2op_msg_, map2op_);

    double roll, pitch, yaw;
    tf2::Matrix3x3(map2op_.getRotation()).getRPY(roll, pitch, yaw);

    ROS_INFO("map -> operator [%lf, %lf, %lf] %lf ago",
        map2op_.getOrigin().x(),
        map2op_.getOrigin().y(),
        yaw,
        (ros::Time::now() - map2op_.stamp_).toSec());

    pos_.position.x = map2op_.getOrigin().x();
    pos_.position.y = map2op_.getOrigin().y();
    pos_.position.z = map2op_.getOrigin().z();
    pos_.orientation.x = map2op_.getRotation().x();
    pos_.orientation.y = map2op_.getRotation().y();
    pos_.orientation.z = map2op_.getRotation().z();
    pos_.orientation.w = map2op_.getRotation().w();

    setOutput<bool>("new_goal", true);
    setOutput<geometry_msgs::Pose>("position", pos_);

    return BT::NodeStatus::SUCCESS;
  }
  else  
  {
    return BT::NodeStatus::FAILURE;
  }


}


}  // namespace vision

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<vision::DetectOperator>("DetectOperator");
}
