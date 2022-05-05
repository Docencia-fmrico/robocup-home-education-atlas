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

#include "dialog_atlas/Get_Name.h"
#include "dialog_atlas/ProvideInfo.h"
#include "dialog_atlas/get_name.h"

#include "ros/ros.h"

namespace dialog_atlas
{


ProvideInfo::ProvideInfo(const std::string& name)
: BT::ActionNodeBase(name, {}), nh_()
{
    sub_name_ = nh_.subscribe("/get_name", 1, &ProvideInfo::nameCallback, this);
    //sub_description = nh_.subscribe("/scan_filtered", 1,&ProvideInfo::descriptionCallback, this); 
    //sub_object_ = nh_.subscribe("/get_name", 1, &ProvideInfo::objectCallback, this);

}


void
ProvideInfo::nameCallback(const dialog_atlas::get_name::ConstPtr& msg)
{
    name_ = msg->name;
}


/*
void
ProvideInfo::descriptionCallback(const dialog_atlas::get_name::ConstPtr& msg)
{
    descrption_ = msg->name;
}

void
ProvideInfo::objectCallback(const dialog_atlas::get_name::ConstPtr& msg)
{
    object_ = msg->name;
}*/


void
ProvideInfo::halt()
{
  ROS_INFO("ProvideInfo halt");
}



BT::NodeStatus
ProvideInfo::tick()
{

  ROS_INFO("Dialog tick");

  Get_Name name_obtained;
  name_obtained.speak(name_);
  return BT::NodeStatus::SUCCESS;
}

   
}  // namespace dialog_atlas


#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<dialog_atlas::ProvideInfo>("ProvideInfo");
}



