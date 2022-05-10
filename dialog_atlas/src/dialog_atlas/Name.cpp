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


#include "behaviortree_cpp_v3/behavior_tree.h"

#include "dialog_atlas/Name.h"
#include "dialog_atlas/Get_Name.h"

#include "dialog_atlas/activate_msg.h"


#include "ros/ros.h"

namespace dialog_atlas
{

Name::Name(const std::string& name)
: BT::ActionNodeBase(name, {}), active_dialog_(true)
{
  sub_ = n_.subscribe("activate_topic", 1, &dialog_atlas::Name::messageCallback, this);
}


void Name::messageCallback(const dialog_atlas::activate_msg::ConstPtr& msg)
{
  //activate = msg->activate;
}



void
Name::halt()
{
  ROS_INFO("Dialog halt");
}

BT::NodeStatus
Name::tick()
{
  ROS_INFO("Dialog tick");

  dialog_atlas::Get_Name forwarder;


  if (forwarder.obtain_name())
  {
    return BT::NodeStatus::SUCCESS;
  }
 
  else
  {
    return BT::NodeStatus::RUNNING;

  }
  
  

}

}  // namespace dialog_atlas

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<dialog_atlas::Name>("Name");
}