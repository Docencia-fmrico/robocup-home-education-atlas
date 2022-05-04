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





#include "ros/ros.h"

#include "behaviortree_cpp_v3/behavior_tree.h"


#include "dialog_atlas/GiveDescription.h"
#include "dialog_atlas/get_name.h"

namespace dialog_atlas
{


GiveDescription::GiveDescription(const std::string& name)
: BT::ActionNodeBase(name, {}), nh_()
{
    sub_name_ = nh_.subscribe("/get_name", 1, &GiveDescription::nameCallback, this);
    //sub_description = nh_.subscribe("/scan_filtered", 1, &BumpGo_Advanced_Laser::laserCallback, this);
}


void
GiveDescription::nameCallback(const dialog_atlas::get_name::ConstPtr& msg)
{
    name_ = msg->name;

}


void
GiveDescription::halt()
{
  ROS_INFO("GiveDescription halt");
}

   
}


#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<dialog_atlas::GiveDescription>("GiveDescription");
}



