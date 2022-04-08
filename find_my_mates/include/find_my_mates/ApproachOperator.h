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

#ifndef FIND_MY_MATES_APPROACHOPERATOR_H
#define FIND_MY_MATES_APPROACHOPERATOR_H

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

#include <move_base_msgs/MoveBaseAction.h>
#include <geometry_msgs/Pose.h>
#include "find_my_mates/BTNavAction.h"
#include <string>

namespace find_my_mates
{
class ApproachOperator : public BTNavAction
{
  public:
    explicit ApproachOperator(const std::string& name,
    const std::string& action_name,
    const BT::NodeConfiguration& config);

    void on_halt() override;
    BT::NodeStatus on_tick() override;
    void on_start() override;
    void on_feedback(const move_base_msgs::MoveBaseFeedbackConstPtr& feedback) override;

    static BT::PortsList providedPorts()
    {
        return { BT::InputPort<geometry_msgs::Pose>("position") };
    }
};
}  // namespace find_my_mates

#endif  // FIND_MY_MATES_APPROACHOPERATOR_H