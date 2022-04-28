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
#include "dialog_atlas/get_name.h"
#include <string>

namespace ph = std::placeholders;
namespace dialog_atlas
{
class Get_Name: public gb_dialog::DialogInterface
{
  public:
   
    Get_Name(): 
    nh_(), state_(IDLE)
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
 
    void noIntentCB(dialogflow_ros_msgs::DialogflowResult result)
    {
      ROS_INFO("[Get_Name] noIntentCB: intent [%s]", result.intent.c_str());
    }


    void getNameIntentCB(dialogflow_ros_msgs::DialogflowResult result)
    {
      ROS_INFO("[Get_Name] getNameIntentCB: intent [%s]", result.intent.c_str());
      for (const auto & param : result.parameters){
        for (const auto & value : param.value){  //obtener la respuesta
        std::cerr << "\t" << value << std::endl;
        }
      }
      speak(result.fulfillment_text);
  
    }


    void checkNameIntentCB(dialogflow_ros_msgs::DialogflowResult result)
    {
      ROS_INFO("[Get_Name] checkNameIntentCB: intent [%s]", result.intent.c_str());
      speak(result.fulfillment_text);
  
    }


    
    void dialog()
    {

      dialog_atlas::get_name person_name;


      switch (state_)
      {
      
        case IDLE:
        
          ros::Duration(0.5).sleep();
          speak("Tell me your name");
          state_ = LISTEN;
          start_ts_ = ros::Time::now();


        case LISTEN:
          
          ROS_INFO("IDEL -> LISTEN");
          listen_ts_  = ros::Time::now();
          while ((ros::Time::now() - start_ts_).toSec() <= WAITING_TIME) 
          {
            listen();
            

          }


          break;

        case SPEAK:

          ROS_INFO("LISTEN -> SPEAK");
          //listen();

          break;
      }

    //pub_name_.publish(person_name);
    
    }


   private:
    ros::NodeHandle nh_;
    ros::Publisher pub_name_;
    ros::Time listen_ts_;
    ros::Time start_ts_;

    int state_;
    static constexpr double WAITING_TIME = 10;
    static const int IDLE = 0;
    static const int LISTEN = 1;
    static const int SPEAK = 2;
    
};
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "get_name_node");

  dialog_atlas::Get_Name forwarder;
  
  ros::Rate loop_rate(20);
  
 
  forwarder.dialog();
  //ros::spin();
  ros::spinOnce();
  ros::Duration(1).sleep();
  loop_rate.sleep();
    
  return 0;
}





