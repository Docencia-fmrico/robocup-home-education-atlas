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
#include "carry_my_luggage_dialog/Get_Luggage.h"

#include "ros/ros.h"
#include <string>

namespace ph = std::placeholders;

namespace carry_my_luggage_dialog
{
  Get_Luggage::Get_Luggage():
  nh_(),keep_listening_(true), stop_(false)

  {
      this->registerCallback(std::bind(&Get_Luggage::noIntentCB, this, ph::_1));

      this->registerCallback(
          std::bind(&Get_Luggage::getLuggageIntentCB, this, ph::_1),
          "Get Luggage Intent"); 
  }


  void Get_Luggage::noIntentCB(dialogflow_ros_msgs::DialogflowResult result)
  {
    ROS_INFO("[Get_Luggage] noIntentCB: intent [%s]", result.intent.c_str());
  }

void Get_Luggage::getLuggageIntentCB(dialogflow_ros_msgs::DialogflowResult result)
  {
    ROS_INFO("[Get_Luggage] getLuggageIntentCB: intent [%s]", result.intent.c_str());
    speak(result.fulfillment_text);

      for (const auto & param : result.parameters){
      for (const auto & value : param.value){  
        luggage_ = value;
        }
      }

    keep_listening_= false;
  
  }


  bool Get_Luggage::choose_luggage()
  {
    
      ros::Duration(0.2).sleep();
      speak("What luggage should I get");

      start_ts_ = ros::Time::now();
      
      if((ros::Time::now() - start_ts_).toSec() < WAITING_TIME) 
      {
        listen();

        if ( keep_listening_ == false)
        {
          stop_ = true;
        }
      
      }

      std::cout << stop_ << "\n";
      return stop_;

  }


  //bool Get_Luggage::stop()
  //{
    //if stop_= 

  //}

}







