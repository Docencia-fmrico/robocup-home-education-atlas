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

#include "dialog_atlas/get_name.h"

class dialog_topic
{
public:
    ros::NodeHandle nh_;
    std::string name; 


    ros::Publisher pub_ = nh_.advertise<dialog_atlas::get_name>("get_name",1);


    void pub_name()
    {
        dialog_atlas::get_name get_name;

        get_name.name = name;
        pub_.publish(get_name);
    }


    void getnameCallback(dialog_topic *name_getter)
    {
        std::cout << name_getter->name;
        name_getter->pub_name();
    }

};



int main(int argc, char** argv)
{
  ros::init(argc, argv, "pub_name_node");

  dialog_topic name_getter;
  ros::Rate loop_rate(20);

  while (ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();

  }

  return 0;
}