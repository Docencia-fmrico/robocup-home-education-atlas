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

#include "dialog_atlas/activate_msg.h"

class activate_topic
{
public:
    ros::NodeHandle nh_;
    bool activate_dialog;


    ros::Publisher pub_ = nh_.advertise<dialog_atlas::activate_msg>("activate_dialog",1);


    void pub_activate_dialog()
    {
        dialog_atlas::activate_msg actv;

        actv.activate = activate_dialog;
        pub_.publish(actv);
    }


    void activateCallback(activate_topic *activate_getter)
    {
        std::cout << activate_getter->activate_dialog;
        activate_getter->pub_activate_dialog();
    }

};



int main(int argc, char** argv)
{
  ros::init(argc, argv, "pub_activate_node");

  activate_topic activate_getter;
  ros::Rate loop_rate(20);

  while (ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();

  }

  return 0;
}