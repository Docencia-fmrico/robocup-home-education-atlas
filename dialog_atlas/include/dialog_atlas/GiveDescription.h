#ifndef GIVE_DESCRIPTION__H
#define GIVE_DESCRIPTION__H

#include <ros/ros.h>
#include "dialog_atlas/get_name.h"

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"



namespace dialog_atlas
{
class GiveDescription: public BT::ActionNodeBase
{
public:

  explicit GiveDescription(const std::string& name);
  
  void nameCallback(const dialog_atlas::get_name::ConstPtr& msg);
  void halt();
  BT::NodeStatus tick();
  //void description();
 

private:


  std::string name_;

  ros::NodeHandle nh_;
  ros::Subscriber sub_name_;
  ros::Subscriber sub_description_;

};
};  // namespace dialog_atlas



#endif
