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

#include "robocup_navigation/GetPosition.h"
#include "robocup_navigation/Position.h"
#include "robocup_navigation/enum_pos.h"
#include "time.h"

namespace robocup_navigation
{

GetPosition::GetPosition(const std::string& name, const BT::NodeConfiguration& config)
: BT::ActionNodeBase(name, config), n_guests_(0), guest_(0), new_goal_(false)
{
  pos_client_ = n_.serviceClient<robocup_navigation::Position>("position");
  
  srand(time(NULL));

  for (int i = 0; i < n_guests_; i++)
    guests[i] = 0;
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

  if (goal_ == GUEST_POS)
  {
    goal_ = ChooseGuest();
  }

  robocup_navigation::Position srv;
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
  return BT::NodeStatus::SUCCESS;
}

int 
GetPosition::ChooseGuest()
{
  if (n_guests_ == 0)
  {
    bool repeat = true;
    do 
    {
      guest_ = rand() % GUEST_3 + GUEST_1;
      ROS_INFO("first guest--------> %i", guest_);
      // checks if the number of the guest is not greater than the last guest
      repeat = guest_ > GUEST_3;
    }
    while (repeat);
    guests[n_guests_] = guest_;
    n_guests_++;
  }
  else if (n_guests_ < MAX_GUESTS)
  {
    new_goal_ = true;
    bool repeated = true;
    do 
    {
      guest_ = rand() % GUEST_3 + GUEST_1;
      ROS_INFO("next guest--------> %i", guest_);
      // checks that the guest is not repeated
      for (int i = 0; i < n_guests_; i++)
      {
        if (guests[i] == guest_ || guest_ > GUEST_3) 
          break;
        repeated = guests[i] != guest_ && i != n_guests_-1;
      }
    }
    while (repeated);
    
    guests[n_guests_] = guest_;
    n_guests_++;
  }
  else
  {
    n_guests_ = 0;

    for (int i = 0; i < n_guests_; i++)
      guests[i] = 0;
  }
  
  return guest_;
}

}  // namespace robocup_navigation

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<robocup_navigation::GetPosition>("GetPosition");
}