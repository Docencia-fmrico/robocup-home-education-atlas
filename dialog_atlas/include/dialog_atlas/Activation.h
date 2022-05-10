

#ifndef ACTIVATION__H
#define ACTIVATION__H

#include <ros/ros.h>



namespace dialog_atlas
{
class Activation
{
public:
  
  Activation();

  void activate(bool activate_dialog);

private:

  ros::NodeHandle nh_;
  ros::Publisher pub_;

};
}  // namespace dialog_atlas



#endif
