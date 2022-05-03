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

#include "vision/DetectPerson.h"

#include "behaviortree_cpp_v3/behavior_tree.h"

namespace vision
{

DetectPerson::DetectPerson(const std::string& name)
: BT::ConditionNode(name, {}), n_()
{
  sub_ = n_.subscribe("bbx_custom_topic", 1, &vision::DetectPerson::messageCallback, this);
}
void
DetectPerson::messageCallback(const vision::bbx_info::ConstPtr& msg){
  last_lecture = msg->header.stamp;
  dist = msg->dist;
}

BT::NodeStatus
DetectPerson::tick()
{
  ROS_INFO("DetectPerson tick");

  ROS_INFO("time since last msg %f",(ros::Time::now() - last_lecture).toSec());
  ROS_INFO("ros time %f",ros::Time::now().toSec());
  ROS_INFO("msg time %f",last_lecture.toSec());
  
  if (((ros::Time::now() - last_lecture).toSec() > 1.0 || last_lecture.toSec() == 0.0) || std::isnan(dist))
  {
    return BT::NodeStatus::FAILURE;
  }
  else  
  {
    return BT::NodeStatus::SUCCESS;
  }
}

}  // namespace vision

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<vision::DetectPerson>("DetectPerson");
}
