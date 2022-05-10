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

#include "movement/GetPosition.h"
#include "movement/Position.h"
#include "enum_pos.h"

namespace movement
{

GetPosition::GetPosition(const std::string& name, const BT::NodeConfiguration& config)
: BT::ActionNodeBase(name, config)
{
  pos_client_ = n_.serviceClient<movement::Position>("position");
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

  goal_ = getInput<int>("goal").value();

  movement::Position srv;
  srv.request.goal = goal_;

  if (pos_client_.call(srv))
  {
    pos_.position.x = srv.response.pos.position.x;
    pos_.position.y = srv.response.pos.position.y;
    pos_.position.z = srv.response.pos.position.z;
    pos_.orientation.x = srv.response.pos.orientation.x;
    pos_.orientation.y = srv.response.pos.orientation.y;
    pos_.orientation.z = srv.response.pos.orientation.z;
    pos_.orientation.w = srv.response.pos.orientation.w;
  }
  else
  {
    ROS_ERROR("Failed to call service distance");
    return BT::NodeStatus::FAILURE;
  }

  setOutput<geometry_msgs::Pose>("position", pos_);
  setOutput<bool>("new_goal", false);
  return BT::NodeStatus::SUCCESS;
}

}  // namespace movement

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<movement::GetPosition>("GetPosition");
}