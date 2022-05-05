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

#ifndef MOVEMENT_CURRENTPOSITION_H
#define MOVEMENT_CURRENTPOSITION_H

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <string>

#include "ros/ros.h"

namespace movement
{

class CurrentPosition : public BT::ActionNodeBase
{
  public:
    explicit CurrentPosition(const std::string& name, const BT::NodeConfiguration& config);
    
    void halt();

    BT::NodeStatus tick();

    static BT::PortsList providedPorts()
    {
        return 
        { 
          BT::OutputPort<geometry_msgs::PoseStamped>("current_pos") 
        };
    }

  private:
    tf::TransformListener listener_;
    geometry_msgs::PoseStamped pos_;
};

}  // namespace movement

#endif  // MOVEMENT_GETPOSITION_H