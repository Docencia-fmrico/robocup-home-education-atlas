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


#include <gb_dialog/DialogInterface.h>
#include <string>

#include "dialog_atlas/get_name.h"
#include "ros/ros.h"

#include "dialog_atlas/Get_Name.h"

namespace ph = std::placeholders;
namespace dialog_atlas
{
  Get_Name::Get_Name(): 
  nh_(), state_(IDLE), keep_listening_(true),keep_listening_2_(true), stop_(false)
  {

    pub_name_ = nh_.advertise<dialog_atlas::get_name>("/get_name",1);

    this->registerCallback(std::bind(&Get_Name::noIntentCB, this, ph::_1));

    this->registerCallback(
      std::bind(&Get_Name::getNameIntentCB, this, ph::_1),
      "Get Name Intent"); 
    
    this->registerCallback(
      std::bind(&Get_Name::checkNameIntentCB, this, ph::_1),
      "Check Name Intent"); 

    
  }
 
  void Get_Name::noIntentCB(dialogflow_ros_msgs::DialogflowResult result)
  {
    ROS_INFO("[Get_Name] noIntentCB: intent [%s]", result.intent.c_str());
  }

  void Get_Name::getNameIntentCB(dialogflow_ros_msgs::DialogflowResult result)
  {
    ROS_INFO("[Get_Name] getNameIntentCB: intent [%s]", result.intent.c_str());
    speak(result.fulfillment_text);

    for (const auto & param : result.parameters){
      param_name_ = result.intent;
      for (const auto & value : param.value){  
        name_ = value;
        }
    }

    
    keep_listening_ = false;
    

  }
   

  void Get_Name::checkNameIntentCB(dialogflow_ros_msgs::DialogflowResult result)
  {
    ROS_INFO("[Get_Name] checkNameIntentCB: intent [%s]", result.intent.c_str());
    speak(result.fulfillment_text);

      for (const auto & param : result.parameters){
      param_name_2_ = result.intent;
      for (const auto & value : param.value){  
        check_ = value;
        }
      }

    if (check_ == "Yes")
    {
      keep_listening_2_ = false;
    }

  }

    
  void Get_Name::dialog()
  {
    
    dialog_atlas::get_name person_name_;


    switch (state_)
    {
    
      case IDLE:
      
        ros::Duration(0.2).sleep();
        speak("Tell me your name");
        state_ = LISTEN;
        ROS_INFO("IDLE -> LISTEN");
        start_ts_ = ros::Time::now();
        

        break;

      case LISTEN:
        
        if ((ros::Time::now() - start_ts_).toSec() < WAITING_TIME) 
        {
          
          listen();
          
          if ( keep_listening_ == false) 
          {
            person_name_.name = name_;
              
            if (keep_listening_2_ == false)
            {
              ROS_INFO("NAME OBTAINED");
              pub_name_.publish(person_name_);
              stop_ = true;
              
            }
          }
          
        }
        
        else
        {
          state_ = SPEAK;
          ROS_INFO("LISTEN -> SPEAK");
        }
        
        break;

      case SPEAK:

        state_ = IDLE;
        break;
    }
    
  }


  bool Get_Name::stop_node()
  {
    return stop_;
  }

  bool Get_Name::obtain_name()
  {

    bool state = false;
    dialog_atlas::Get_Name forwarder;
    ros::Rate loop_rate(20);
    
    while(ros::ok())
    {
      forwarder.dialog();
      ros::spinOnce();
      loop_rate.sleep();
    
      if (forwarder.stop_node() == true)
      { 
        state = true; 
        break;
      }
    
    }

    return state;

  }

    
}
