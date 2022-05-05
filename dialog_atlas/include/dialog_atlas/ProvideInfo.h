#ifndef PROVIDEINFO__H
#define PROVIDEINFO__H


#include <string>
#include <ros/ros.h>

#include "dialog_atlas/get_name.h"

#include "behaviortree_cpp_v3/behavior_tree.h"
#include "behaviortree_cpp_v3/bt_factory.h"


namespace dialog_atlas
{
class ProvideInfo: public BT::ActionNodeBase
{
public:

  explicit ProvideInfo(const std::string& name);
  
  void nameCallback(const dialog_atlas::get_name::ConstPtr& msg);
  void halt();
  BT::NodeStatus tick();
 

private:


  std::string name_;
  std::string description_;
  std::string object_;

  ros::NodeHandle nh_;
  ros::Subscriber sub_name_;
  ros::Subscriber sub_description_;
  ros::Subscriber sub_object_;

};
};  // namespace dialog_atlas



#endif
