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

#include "find_my_mates/GetDescription.h"

namespace find_my_mates
{

GetDescription::GetDescription(const std::string& name, const BT::NodeConfiguration & config)
: BT::ActionNodeBase(name, config), n_()
{
  sub_ = n_.subscribe("bbx_ropa", 1, &GetDescription::callback, this);
}

void 
GetDescription::callback(const vision::ropa_hsv::ConstPtr msg)
{
    s_hupper_ = msg->shirt.hupper;
    s_hlower_ = msg->shirt.hlower;
    s_supper_ = msg->shirt.supper;
    s_slower_ = msg->shirt.slower;
    s_vupper_ = msg->shirt.vupper;
    s_vlower_ = msg->shirt.vlower;
    t_hupper_ = msg->trousers.hupper;
    t_hlower_ = msg->trousers.hlower;
    t_supper_ = msg->trousers.supper;
    t_slower_ = msg->trousers.slower;
    t_vupper_ = msg->trousers.vupper;
    t_vlower_ = msg->trousers.vlower;
}

void
GetDescription::halt()
{
  ROS_INFO("GetDescription halt");
  s_hupper_ = s_hlower_ = s_supper_ = s_slower_ = s_vupper_ = s_vlower_ = 0;
  t_hupper_ = t_hlower_ = t_supper_ = t_slower_ = t_vupper_ = t_vlower_ = 0;
}

BT::NodeStatus
GetDescription::tick()
{
  ROS_INFO("GetDescription tick");

  if (t_vupper_)  
  {
    ROS_INFO("Shirt: %i, %i, %i, %i, %i, %i", s_hupper_, s_hlower_, s_supper_, s_slower_, s_vupper_, s_vlower_);
    ROS_INFO("Trousers: %i, %i, %i, %i, %i, %i", t_hupper_, t_hlower_, t_supper_, t_slower_, t_vupper_, t_vlower_);
    return BT::NodeStatus::SUCCESS;
  }
  else
  {
    ROS_INFO("Shirt: %i, %i, %i, %i, %i, %i", s_hupper_, s_hlower_, s_supper_, s_slower_, s_vupper_, s_vlower_);
    ROS_INFO("Trousers: %i, %i, %i, %i, %i, %i", t_hupper_, t_hlower_, t_supper_, t_slower_, t_vupper_, t_vlower_);
    return BT::NodeStatus::RUNNING;
  }

  //setOutput<string>("description", );
}

}  // namespace find_my_mates

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<find_my_mates::GetDescription>("GetDescription");
}
