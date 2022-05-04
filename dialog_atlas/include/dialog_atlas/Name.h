
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

#ifndef NAME_H
#define NAME_H

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"


#include <string>
#include "ros/ros.h"

#include "dialog_atlas/activate_msg.h"


namespace dialog_atlas
{

class Name : public BT::ActionNodeBase
{
  public:
    explicit Name(const std::string& name);

    void messageCallback(const dialog_atlas::activate_msg::ConstPtr& msg);
    void halt();

    BT::NodeStatus tick();

  private:

    BT::NodeStatus status_;

    ros::Subscriber sub_;
    ros::NodeHandle n_;

    bool activate;

};

}  // namespace movement

#endif  // Name_H