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

#include "ros/ros.h"
#include <string>

namespace ph = std::placeholders;

namespace carry_my_luggage_dialog
{
class Get_Luggage: public gb_dialog::DialogInterface
{

public:
    Get_Luggage():
    nh_(), state_(IDLE), keep_listening_(true), stop_(false) 

    {
        this->registerCallback(std::bind(&Get_Luggage::noIntentCB, this, ph::_1));

        this->registerCallback(
            std::bind(&Get_Luggage::getLuggageIntentCB, this, ph::_1),
            "Get Luggage Intent"); 
    }

 
    void noIntentCB(dialogflow_ros_msgs::DialogflowResult result)
    {
      ROS_INFO("[Get_Luggage] noIntentCB: intent [%s]", result.intent.c_str());
    }

    void getLuggageIntentCB(dialogflow_ros_msgs::DialogflowResult result)
    {
      ROS_INFO("[Get_Luggage] getLuggageIntentCB: intent [%s]", result.intent.c_str());
      speak(result.fulfillment_text);

      for (const auto & param : result.parameters){
        param_name_ = result.intent;
        for (const auto & value : param.value){  
          luggage_ = value;
          }
      }

      if (param_name_ == "Get Luaggage Intent")
      {
        keep_listening_ = false;
      }

    }


    void dialog()
    {
    
      switch (state_)
      {
      
        case IDLE:
        
          ros::Duration(0.2).sleep();
          speak("What luggage should I get");
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
              
              stop_ = true;
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


    bool stop_node()
    {
      return stop_;
    }
   


private:
    ros::NodeHandle nh_;

    ros::Time start_ts_;

    
    std::string luggage_;
    std::string param_name_; 
    
    

    bool keep_listening_;
    bool stop_;
    
   
    int state_;

    static constexpr double WAITING_TIME = 15;
    static const int IDLE = 0;
    static const int LISTEN = 1;
    static const int SPEAK = 2; 
    
};
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "get_luggage_node");

  carry_my_luggage_dialog::Get_Luggage forwarder;
  

  ros::Rate loop_rate(20);
    
  while(ros::ok())
  {
    forwarder.dialog();
    ros::spinOnce();
    loop_rate.sleep();
  
    if (forwarder.stop_node() == true)
    { 
      ros::shutdown(); 
    }
  
  }
    
  return 0;
}



