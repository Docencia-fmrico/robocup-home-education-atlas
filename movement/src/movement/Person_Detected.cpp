

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

#include "movement/Person_Detected.h"
#include "movement/bbx_info.h"
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

namespace movement
{

Person_Detected::Person_Detected(const std::string& name)
: BT::ConditionNode(name,{}), n_()
{
  sub_ = n_.subscribe("bbx_person", 1, &movement::Person_Detected::messageCallback, this);
}
void
Person_Detected::messageCallback(const movement::bbx_info::ConstPtr& msg){
  last_lecture = msg->header.stamp;
  dist = msg->dist;
  ROS_INFO("xmax %d, xmin %d\n",msg->xmax, msg->xmin);
}

BT::NodeStatus
Person_Detected::tick()
{
  if ( (ros::Time::now() - last_lecture).toSec() > 1.0 || std::isnan(dist))
  {
    ROS_INFO("time since las msg %f",(ros::Time::now() - last_lecture).toSec());
    ROS_INFO("ros time %f",ros::Time::now().toSec());
    ROS_INFO("msg time %f",last_lecture.toSec());
    return BT::NodeStatus::FAILURE;
  }
  else  
  {
    ROS_INFO("persona detectada");
    return BT::NodeStatus::SUCCESS;
  }
}

}  // namespace movement

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<movement::Person_Detected>("Person_Detected");
}