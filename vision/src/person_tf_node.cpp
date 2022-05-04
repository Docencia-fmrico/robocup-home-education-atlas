// Copyright 2019 Intelligent Robotics Lab
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

#include <ros/ros.h>
#include <ros/console.h>

#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/octree/octree_search.h>
#include <pcl/octree/octree.h>
#include <pcl/io/ply_io.h>
#include <pcl_ros/transforms.h>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <tf/message_filter.h>
#include <message_filters/subscriber.h>

#include <std_msgs/Float32.h>
#include <sensor_msgs/PointCloud2.h>

#include <boost/algorithm/string.hpp>

#include "vision/bbx_info.h"


class RGBDtf
{
public:
  RGBDtf(): objectFrameId_("/person/0")
  , workingFrameId_("/map")
  , cameraTopicId_("/cloud_filtered/0")
  {
    ros::NodeHandle private_nh("~");
    private_nh.param("object_id", objectFrameId_, objectFrameId_);
    private_nh.param("cloud_id", cameraTopicId_, cameraTopicId_);

    ROS_INFO("object_id: [%s]", objectFrameId_.c_str());
    ROS_INFO("cloud_id : [%s]", cameraTopicId_.c_str());

    pointCloudSub_ =
      new message_filters::Subscriber<sensor_msgs::PointCloud2> (nh_, cameraTopicId_, 5);
    tfPointCloudSub_ =
      new tf::MessageFilter<sensor_msgs::PointCloud2> (*pointCloudSub_, tfListener_, workingFrameId_, 5);
    tfPointCloudSub_->registerCallback(boost::bind(&RGBDtf::cloudCB, this, _1));


    sub_ = nh_.subscribe("bbx_custom_topic", 1, &RGBDtf::bbxPersonCB, this);
  }

  void bbxPersonCB(const vision::bbx_info::ConstPtr& msg)
  {
    ROS_INFO("Message: [%i,%i]", msg->px, msg->px);
    person_px_ = msg->px;
    person_py_ = msg->py;
    person_pz_ = 0;
  }

  void cloudCB(const sensor_msgs::PointCloud2::ConstPtr& cloud_in)
  {
    sensor_msgs::PointCloud2 cloud;

    try
    {
      pcl_ros::transformPointCloud(workingFrameId_, *cloud_in, cloud, tfListener_);
    }
    catch(tf::TransformException & ex)
    {
      ROS_ERROR_STREAM("Transform error of sensor data: " << ex.what() << ", quitting callback");
      return;
    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pcrgb(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromROSMsg(cloud, *pcrgb);

    if (!std::isnan(person_px_) && !std::isnan(person_py_) && !std::isnan(person_pz_))
    {
      tf::StampedTransform transform;
      transform.setOrigin(tf::Vector3(person_px_, person_py_, person_pz_));
      transform.setRotation(tf::Quaternion(0.0, 0.0, 0.0, 1.0));

      transform.stamp_ = ros::Time::now();
      transform.frame_id_ = workingFrameId_;
      transform.child_frame_id_ = objectFrameId_;

      try
      {
        tfBroadcaster_.sendTransform(transform);
      }
      catch(tf::TransformException& ex)
      {
        ROS_ERROR_STREAM("Transform error of sensor data: " << ex.what() << ", quitting callback");
        return;
      }
    }
  }


private:
  ros::NodeHandle nh_;

  tf::TransformBroadcaster tfBroadcaster_;
  tf::TransformListener tfListener_;

  tf::MessageFilter<sensor_msgs::PointCloud2>* tfPointCloudSub_;
  message_filters::Subscriber<sensor_msgs::PointCloud2>* pointCloudSub_;

  std::string objectFrameId_;
  std::string workingFrameId_;
  std::string cameraTopicId_;

  ros::Subscriber sub_;

  int person_px_, person_py_, person_pz_;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "rgbd_tf");
  RGBDtf rgbdrf;
  ros::spin();
  return 0;
}
