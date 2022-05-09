#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

#include <cmath>
#include <vector>
#include <iostream>

#include <sensor_msgs/Image.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include "fsm_robocup/bbx_info.h"
#include "fsm_robocup/ropa_hsv.h"
#include "std_msgs/Header.h"

class bbx_info{
public:
  ros::NodeHandle nh;
  int px, py;
  int bbx_xmin = 0;
  int bbx_ymin = 0;
  int bbx_xmax = 0;
  int bbx_ymax = 0;

  float dist;
  
  std::string color_camiseta;
  std::string color_pantalon;

  std_msgs::Header header;
  ros::Publisher pub = nh.advertise<fsm_robocup::bbx_info>("bbx_person",1);
  ros::Publisher pub_ropa = nh.advertise<fsm_robocup::ropa_hsv>("bbx_ropa",1);

  void publicar()
  {
    fsm_robocup::bbx_info bbx_info;

    bbx_info.header.stamp = ros::Time::now();
    bbx_info.dist = dist;
    bbx_info.px = px;
    bbx_info.py = py;
    bbx_info.xmax = bbx_xmax;
    bbx_info.xmin = bbx_xmin;
    pub.publish(bbx_info);
  }

  void publicar_ropa()
  {
    fsm_robocup::ropa_hsv ropa_hsv;

    ropa_hsv.header.stamp = ros::Time::now();
    ropa_hsv.color_camiseta = color_camiseta;
    ropa_hsv.color_pantalon = color_pantalon;
    pub_ropa.publish(ropa_hsv);
  }

};

void callback_bbx(const sensor_msgs::ImageConstPtr& image, const darknet_ros_msgs::BoundingBoxesConstPtr& boxes, bbx_info *mensajero)
{
  cv_bridge::CvImagePtr img_ptr_depth;
  std::string str_person ("person");

  int object_center_x, object_center_y;
  try{
      img_ptr_depth = cv_bridge::toCvCopy(*image, sensor_msgs::image_encodings::TYPE_32FC1);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception:  %s", e.what());
    return;
  }
  for (const auto & box : boxes->bounding_boxes) {
    if(str_person.compare(box.Class) == 0 && box.probability > 0.7){
      mensajero->px = (box.xmax + box.xmin) / 2;
      mensajero->py = (box.ymax + box.ymin) / 2;
      mensajero->bbx_xmin = box.xmin;
      mensajero->bbx_ymin = box.ymin;
      mensajero->bbx_xmax = box.xmax;
      mensajero->bbx_ymax = box.ymax;

      mensajero->dist = img_ptr_depth->image.at<float>(cv::Point(mensajero->px, mensajero->py))* 0.001f;//* 0.001f
      std::cerr << box.Class << " at (" << mensajero->dist <<"px: "<< mensajero->px << "py: "<< mensajero->py << std::endl;
      mensajero->publicar();
    }
  }
}

std::string nombrar_color(int color){

  std::string color_name;
  switch (color){
    case 0:
      color_name = "rojo";
      break;
    case 1:
      color_name = "naranja";
      break;
    case 2:
      color_name = "azul";
      break;
  }
  return color_name;
}

  void detectar_color(cv::Mat img_ptr_rgb, bbx_info *mensajero){
    cv::Mat rgb, hsv, mask_rojo, mask_rojo_vis;
    cv::Mat mask_naranja, mask_naranja_vis;
    cv::Mat mask_azul, mask_azul_vis;

    rgb = cv::Mat(img_ptr_rgb);
    cv::cvtColor(rgb, hsv, cv::COLOR_BGR2HSV);

    mask_rojo_vis = cv::Mat::zeros( rgb.size(), CV_8UC3 );
    mask_naranja_vis = cv::Mat::zeros( rgb.size(), CV_8UC3 );
    mask_azul_vis = cv::Mat::zeros( rgb.size(), CV_8UC3 );

    cv::inRange(hsv, cv::Scalar(170, 100, 20, 0), cv::Scalar(179, 255, 255, 0), mask_rojo);
    cv::bitwise_and(rgb, rgb, mask_rojo_vis, mask_rojo);

    cv::inRange(hsv, cv::Scalar(0, 100, 20, 0), cv::Scalar(15, 255, 255, 0), mask_naranja);
    cv::bitwise_and(rgb, rgb, mask_naranja_vis, mask_naranja);

    cv::inRange(hsv, cv::Scalar(75, 100, 20, 0), cv::Scalar(140, 255, 255, 0), mask_azul);
    cv::bitwise_and(rgb, rgb, mask_azul_vis, mask_azul);


    std::vector<std::vector<cv::Point> > contours;
    std::vector<cv::Vec4i> jerarquia;
    std::vector<cv::Mat> all_masks = {mask_rojo,mask_naranja,mask_azul};
    int max_area_mask, second_max_area_mask, contour_max_index, second_contour_max_index;
    max_area_mask = 0;
    second_max_area_mask = 0;
    contour_max_index = 0;
    second_contour_max_index = 0;
    double max_area_size = 0;
    double second_max_area_size = 0;

    double y_big, y_2big;

    cv::Mat drawing_1, drawing_2;

    for(int i = 0; i < 3; i++){

      findContours( all_masks[i], contours, jerarquia, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE, cv::Point(0,0));
      std::vector<cv::Moments> mu(contours.size() );
      for( int k = 0; k < contours.size(); k++ ){
        mu[k] = cv::moments( contours[k] );
      }
      for( int j = 0; j < contours.size(); j++ )
      {
        if(cv::contourArea(contours[j]) > max_area_size){
          max_area_mask = i;
          contour_max_index = j;
          max_area_size = cv::contourArea(contours[j]);
          drawing_1 = cv::Mat::zeros( rgb.size(), CV_8UC3 );
          cv::drawContours( drawing_1, contours, j, cv::Scalar(255,255,255), -1, cv::LINE_8);
          y_big = static_cast<int>(mu[j].m01 / (mu[i].m00 + 1e-5));

        }else if(cv::contourArea(contours[j]) > second_max_area_size && cv::contourArea(contours[j]) < max_area_size){
          second_max_area_mask = i;
          second_contour_max_index = j;
          second_max_area_size = cv::contourArea(contours[j]);
          drawing_2 = cv::Mat::zeros( rgb.size(), CV_8UC3 );
          cv::drawContours( drawing_2, contours, j, cv::Scalar(255,255,255), -1, cv::LINE_8);
          y_2big = static_cast<double>(mu[j].m01 / (mu[i].m00 + 1e-5));
        }
      }
    }

    std::string color_name_big = nombrar_color(max_area_mask);
    std::string color_name_2big = nombrar_color(second_max_area_mask);


    mensajero->color_camiseta = color_name_big;
    mensajero->color_pantalon = color_name_2big;

    if( y_big > y_2big){
      mensajero->color_camiseta = color_name_2big;
      mensajero->color_pantalon = color_name_big;
    }

    cv::Mat result, result_drawing;
    cv::add(drawing_1, drawing_2, result_drawing);
    cv::bitwise_and(rgb,result_drawing,result);
    cv::imshow( "Contours", result );
    cv::namedWindow( "rojo", cv::WINDOW_AUTOSIZE );
    cv::imshow( "rojo", mask_rojo_vis);
    cv::namedWindow( "naranja", cv::WINDOW_AUTOSIZE );
    cv::imshow( "naranja", mask_naranja_vis);
    cv::namedWindow( "azul", cv::WINDOW_AUTOSIZE );
    cv::imshow( "azul", mask_azul_vis);

    std::cout << "camiseta " << mensajero->color_camiseta << " y pantalon " << mensajero->color_pantalon << std::endl;
    mensajero->publicar_ropa();
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

    int ancho = abs(mensajero->bbx_xmax - mensajero->bbx_xmin);
    int alto = abs(mensajero->bbx_ymax - mensajero->bbx_ymin);
    ROS_INFO("%d, %d", ancho, alto);
    
    //recortamos la imagen de la persona
    if(ancho > 0 && alto > 0){
      cv::Rect bbx_rect = cv::Rect(mensajero->bbx_xmin,mensajero->bbx_ymin,ancho,alto);
      cv::Mat bbx_img = cv::Mat(img_ptr_rgb->image, bbx_rect);
      cv::namedWindow( "person", cv::WINDOW_AUTOSIZE );
      cv::imshow( "person", bbx_img);
      detectar_color(bbx_img, mensajero);
    }
    
    cv::namedWindow( "complete image", cv::WINDOW_AUTOSIZE );
    cv::imshow( "complete image", img_ptr_rgb->image);
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
  message_filters::Synchronizer<MySyncPolicy_bbx> sync_bbx(MySyncPolicy_bbx(20), image_depth_sub, bbx_sub);
  message_filters::Synchronizer<MySyncPolicy_bbx> sync_bbx_rgb(MySyncPolicy_bbx(20), image_rgb_sub, bbx_sub);

  sync_bbx.registerCallback(boost::bind(&callback_bbx, _1, _2, &mensajero));
  sync_bbx_rgb.registerCallback(boost::bind(&rgb_callback, _1, _2, &mensajero));

  while (ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();

  }

  return 0;
}