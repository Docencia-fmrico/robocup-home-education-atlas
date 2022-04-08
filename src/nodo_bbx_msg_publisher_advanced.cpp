#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

#include <cmath>
#include <vector>

#include <sensor_msgs/Image.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include "fsm_robocup/bbx_info.h"
#include "std_msgs/Header.h"

class bbx_info{
public:
  ros::NodeHandle nh;
  int px, py, bbx_xmin, bbx_ymin;
  float dist;
  std_msgs::Header header;
  ros::Publisher pub = nh.advertise<fsm_robocup::bbx_info>("bbx_custom_topic",1);

  void publicar()
  {
    fsm_robocup::bbx_info bbx_info;

    bbx_info.header.stamp = ros::Time::now();
    bbx_info.dist = dist;
    bbx_info.px = px;
    bbx_info.py = py;
    pub.publish(bbx_info);
  }

};

void callback_bbx(const sensor_msgs::ImageConstPtr& image, const darknet_ros_msgs::BoundingBoxesConstPtr& boxes, bbx_info *mensajero)
  {
    cv_bridge::CvImagePtr img_ptr_depth;
    std::string str_person ("person");
    try{
        img_ptr_depth = cv_bridge::toCvCopy(*image, sensor_msgs::image_encodings::TYPE_32FC1);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception:  %s", e.what());
      return;
    }
    for (const auto & box : boxes->bounding_boxes) {
      if(str_person.compare(box.Class) == 0 && box.probability > 0.8){
        mensajero->px = (box.xmax + box.xmin) / 2;
        mensajero->py = (box.ymax + box.ymin) / 2;
        mensajero->bbx_xmin = box.xmin;
        mensajero->bbx_ymin = box.ymin;

        mensajero->dist = img_ptr_depth->image.at<float>(cv::Point(mensajero->px, mensajero->py))* 0.001f;//* 0.001f

        std::cerr << box.Class << " at (" << mensajero->dist <<"px: "<< mensajero->px << "py: "<< mensajero->py << std::endl;
        mensajero->publicar();
      }
    }

    cv::namedWindow( "depth", cv::WINDOW_AUTOSIZE );
    cv::imshow( "depth", img_ptr_depth->image);
    cv::waitKey( 30 );
  }

  void rgb_callback(const sensor_msgs::ImageConstPtr& image, const darknet_ros_msgs::BoundingBoxesConstPtr& boxes, bbx_info *mensajero)
  {
    cv_bridge::CvImagePtr img_ptr_rgb;

    std::string str_person ("person");
    try{
        img_ptr_rgb = cv_bridge::toCvCopy(*image, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception:  %s", e.what());
      return;
    }
    cv::Rect bbx_rect = cv::Rect(mensajero->bbx_xmin,mensajero->bbx_ymin,100,300);
    cv::Mat bbx_img = cv::Mat(img_ptr_rgb->image, bbx_rect);

    std::cerr << " colors px: "<< mensajero->px << "py: "<< mensajero->py << std::endl;
    cv::namedWindow( "complete image", cv::WINDOW_AUTOSIZE );
    cv::imshow( "complete image", img_ptr_rgb->image);

    cv::Mat hsv;
    
    cv::cvtColor(bbx_img,hsv, cv::COLOR_BGR2HSV);

    cv::namedWindow( "rgb", cv::WINDOW_AUTOSIZE );
    std::vector<cv::Mat> channels;
    cv::split(hsv, channels);

    double h_min, h_max, s_min, s_max, v_min, v_max;
    cv::minMaxLoc(channels[0], &h_min, &h_max, nullptr, nullptr);
    cv::minMaxLoc(channels[1], &s_min, &s_max, nullptr, nullptr);
    cv::minMaxLoc(channels[2], &v_min, &v_max, nullptr, nullptr);
    
    ROS_INFO("%f, %f, %f, %f, %f, %f", h_min, h_max, s_min, s_max, v_min, v_max);
    cv::imshow( "rgb", bbx_img);
    cv::waitKey( 30 );
  }

int main(int argc, char** argv)
{
  ros::init(argc, argv, "vision_node");

  bbx_info mensajero;
  ros::Rate loop_rate(20);
  
  message_filters::Subscriber<sensor_msgs::Image> image_depth_sub(mensajero.nh, "/camera/depth/image_raw", 1);
  message_filters::Subscriber<darknet_ros_msgs::BoundingBoxes> bbx_sub(mensajero.nh, "/darknet_ros/bounding_boxes", 1);
  message_filters::Subscriber<sensor_msgs::Image> image_rgb_sub(mensajero.nh, "/camera/rgb/image_raw", 1);


  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, darknet_ros_msgs::BoundingBoxes> MySyncPolicy_bbx;
  message_filters::Synchronizer<MySyncPolicy_bbx> sync_bbx(MySyncPolicy_bbx(10), image_depth_sub, bbx_sub);
  message_filters::Synchronizer<MySyncPolicy_bbx> sync_bbx_rgb(MySyncPolicy_bbx(10), image_rgb_sub, bbx_sub);

  sync_bbx.registerCallback(boost::bind(&callback_bbx, _1, _2, &mensajero));
  sync_bbx_rgb.registerCallback(boost::bind(&rgb_callback, _1, _2, &mensajero));

  while (ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();

  }

  return 0;
}