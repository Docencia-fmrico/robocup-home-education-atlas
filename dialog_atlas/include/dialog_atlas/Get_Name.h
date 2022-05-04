#ifndef GIVE_NAME__H
#define GIVE_NAME__H

#include <gb_dialog/DialogInterface.h>
#include <string>

#include "dialog_atlas/get_name.h"
#include "ros/ros.h"

namespace dialog_atlas
{
class Get_Name: public gb_dialog::DialogInterface
{
public:


    Get_Name();
    void noIntentCB(dialogflow_ros_msgs::DialogflowResult result);
    void getNameIntentCB(dialogflow_ros_msgs::DialogflowResult result);
    void checkNameIntentCB(dialogflow_ros_msgs::DialogflowResult result);

    void dialog();
    bool stop_node();
    

 private:
    ros::NodeHandle nh_;
    ros::Publisher pub_name_;
    ros::Subscriber get_name;
    ros::Time start_ts_;

    
    std::string name_;
    std::string check_;
    std::string param_name_; 
    std::string param_name_2_;
    
    

    bool keep_listening_;
    bool keep_listening_2_;
    bool stop_;
    
   

    int state_;

    static constexpr double WAITING_TIME = 15;
    static const int IDLE = 0;
    static const int LISTEN = 1;
    static const int SPEAK = 2; 

};
};  // namespace dialog_atlas



#endif
