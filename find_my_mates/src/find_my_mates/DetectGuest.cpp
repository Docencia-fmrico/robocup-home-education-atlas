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


#include <string>

#include "find_my_mates/DetectGuest.h"

#include "behaviortree_cpp_v3/behavior_tree.h"

#include "ros/ros.h"

namespace find_my_mates
{

DetectGuest::DetectGuest(const std::string& name, const BT::NodeConfiguration & config)
: BT::ActionNodeBase(name, config),
  image_depth_sub(n_, "/camera/depth/image_raw", 1), bbx_sub(n_, "/darknet_ros/bounding_boxes", 1),
  sync_bbx(MySyncPolicy_bbx(10), image_depth_sub, bbx_sub), detected_(false)
{
    sync_bbx.registerCallback(boost::bind(&DetectGuest::callback, this, _1, _2));
}

void
DetectGuest::callback(const sensor_msgs::ImageConstPtr& image, const darknet_ros_msgs::BoundingBoxesConstPtr& boxes)
{
 cv_bridge::CvImagePtr img_ptr_depth;

  try 
  {
      img_ptr_depth = cv_bridge::toCvCopy(*image, sensor_msgs::image_encodings::TYPE_32FC1);
  }
  catch (cv_bridge::Exception& e)
  {
      ROS_ERROR("cv_bridge exception:  %s", e.what());
      return;
  }
  
  std::string str_person ("person");
  for (const auto & box : boxes->bounding_boxes) 
  {
    if(str_person.compare(box.Class) == 0 && box.probability >= 0.8)
    {
      detected_ = true;
      int px = (box.xmax + box.xmin) / 2;
      int py = (box.ymax + box.ymin) / 2;
      float dist = img_ptr_depth->image.at<float>(cv::Point(px,py)) * 0.001f;
      
      std::cerr << box.Class << " at " << dist <<" px: "<< px << " py: "<< py << std::endl;
    }
  }
}

void
DetectGuest::halt()
{
  ROS_INFO("DetectGuest halt");
}

BT::NodeStatus
DetectGuest::tick()
{
  ROS_INFO("DetectGuest tick");

  if (detected_)
  {
    return BT::NodeStatus::SUCCESS;
  }
  else
  {
    return BT::NodeStatus::FAILURE;
  }

  // setOutput<geometry_msgs::Pose>("position", pos_);
}

}  // namespace find_my_mates

#include "behaviortree_cpp_v3/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<find_my_mates::DetectGuest>("DetectGuest");
}
