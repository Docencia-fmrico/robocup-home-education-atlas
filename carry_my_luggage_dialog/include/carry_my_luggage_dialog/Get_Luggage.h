#ifndef GIVE_LUGGAGE__H
#define GIVE_LUGGAGE__H

#include <gb_dialog/DialogInterface.h>
#include <string>

#include "ros/ros.h"

namespace carry_my_luggage_dialog
{
class Get_Luggage: public gb_dialog::DialogInterface
{
public:

    Get_Luggage();
    void noIntentCB(dialogflow_ros_msgs::DialogflowResult result);
    void getLuggageIntentCB(dialogflow_ros_msgs::DialogflowResult result);

    bool choose_luggage();
    bool stop();
    

 private:
    ros::NodeHandle nh_;

    ros::Time start_ts_;

    
    std::string luggage_;

 
    bool keep_listening_;
    bool stop_;

    static constexpr double WAITING_TIME = 15;

};
};  // namespace carry_my_luggage_dialog



#endif
