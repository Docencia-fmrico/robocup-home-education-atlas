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

#ifndef NAVEGACION_ROBOCUP_PRUEBA_H
#define NAVEGACION_ROBOCUP_PRUEBA_H

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"

#include <string>

#include "ros/ros.h"

namespace navegacion_robocup
{
class Prueba : public BT::ActionNodeBase
{
  public:
    explicit Prueba(const std::string& name, const BT::NodeConfiguration& config);

    void callback();
    
    void halt();

    BT::NodeStatus tick();

    static BT::PortsList providedPorts()
    {
        return 
        { 

        };
    }

  private:
    ros::NodeHandle n_;
};

}  // namespace navegacion_robocup

#endif  // NAVEGACION_ROBOCUP_PRUEBA_H