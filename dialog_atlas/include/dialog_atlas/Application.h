

#ifndef APPLICATION__H
#define APPLICATION__H

#include <ros/ros.h>



namespace dialog_atlas
{
class Application
{
public:
  
  Application();

  void activate(bool activate_dialog);

private:

  ros::NodeHandle nh_;
  ros::Publisher pub_;

};
}  // namespace dialog_atlas



#endif
