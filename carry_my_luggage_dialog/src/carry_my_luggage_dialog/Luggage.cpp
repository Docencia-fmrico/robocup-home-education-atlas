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

#include "carry_my_luggage_dialog/Luggage.h"
#include "carry_my_luggage_dialog/Get_Luggage.h"

#include "dialog_atlas/activate_msg.h"


#include "ros/ros.h"

namespace carry_my_luggage_dialog
{

Luggage::Luggage(const std::string& Luggage)
: BT::ActionNodeBase(Luggage, {}), status_(BT::NodeStatus::FAILURE)
{
  sub_ = n_.subscribe("activate_topic", 1, &carry_my_luggage_dialog::Luggage::messageCallback, this);
}


void Luggage::messageCallback(const dialog_atlas::activate_msg::ConstPtr& msg)
{
  activate = msg->activate;
}



void
Luggage::halt()
{
  ROS_INFO("Dialog halt");
}

BT::NodeStatus
Luggage::tick()
{
  ROS_INFO("Dialog tick");

  carry_my_luggage_dialog::Get_Luggage forwarder;
  
  //if (activate == true)
  //{
    
  if (forwarder.choose_luggage() == false)
  {
      status_ = BT::NodeStatus::RUNNING;
  }

  else
  {
      status_ = BT::NodeStatus::SUCCESS;
  }
   
  
  //}

  return status_;
}

}  // Luggagespace carry_my_luggage_dialog

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<carry_my_luggage_dialog::Luggage>("Luggage");
}
