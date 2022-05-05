#include <gb_dialog/DialogInterface.h>
#include "carry_my_luggage_dialog/Get_Luggage.h"

#include "ros/ros.h"
#include <string>

 int main(int argc, char** argv)
{
  ros::init(argc, argv, "get_luggage_node");

  carry_my_luggage_dialog::Get_Luggage forwarder;
 

  forwarder.choose_luggage();
  ros::spinOnce();
  
    
  return 0;
}
