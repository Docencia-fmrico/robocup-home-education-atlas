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

#ifndef FIND_MY_MATES_GETDESCRIPTION_H
#define FIND_MY_MATES_GETDESCRIPTION_H

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

#include <vision/ropa_hsv.h>
#include <string>

#include "ros/ros.h"

namespace find_my_mates
{
class GetDescription : public BT::ActionNodeBase
{
  public:
    explicit GetDescription(const std::string& name, const BT::NodeConfiguration& config);

    void callback(const vision::ropa_hsv::ConstPtr msg);
    
    void halt();

    BT::NodeStatus tick();

    static BT::PortsList providedPorts()
    {
        return 
        { 
            //BT::OutputPort<string>("description"),
        };
    }

  private:
    ros::NodeHandle n_;
    ros::Subscriber sub_;
    
    int s_hupper_, s_hlower_, s_supper_, s_slower_, s_vupper_, s_vlower_;
    int t_hupper_, t_hlower_, t_supper_, t_slower_, t_vupper_, t_vlower_;
};
}  // namespace find_my_mates

#endif  // FIND_MY_MATES_GETDESCRIPTION_H