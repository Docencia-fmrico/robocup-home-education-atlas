#include <gb_dialog/DialogInterface.h>
#include "dialog_atlas/Get_Name.h"

#include "ros/ros.h"
#include <string>

 int main(int argc, char** argv)
{
  ros::init(argc, argv, "get_name_node");

  dialog_atlas::Get_Name forwarder;
 

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
