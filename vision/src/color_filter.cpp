#include <opencv2/opencv.hpp>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>

using namespace std;

int main( int argc, char** argv ) {

  cv::namedWindow( "Example 2-10", cv::WINDOW_AUTOSIZE );

  cv::VideoCapture cap;
  if (argc==1) {
    cap.open(0);           // open the first camera
  } else {
    cap.open(argv[1]);     //abre el video que le hayas indicado
  }
  if( !cap.isOpened() ) {  // check if we succeeded
    std::cerr << "Couldn't open capture." << std::endl;
    return -1;
  }
  cv::Mat rgb, hsv, mask_blanco, mask_blanco_vis;
  for(;;) {
    cap >> rgb;

    if( rgb.empty() ) break;             // Ran out of film

    cv::cvtColor(rgb, hsv, cv::COLOR_BGR2HSV);
    cv::inRange(hsv, cv::Scalar(0, 0, 80, 0), cv::Scalar(180, 255, 255, 0), mask_blanco);
    cv::bitwise_and(rgb, rgb, mask_blanco_vis, mask_blanco);
    cv::namedWindow( "blanco", cv::WINDOW_AUTOSIZE );//mostramos la imagen de la persona
    cv::imshow( "blanco", mask_blanco_vis);
    cv::waitKey( 30 );
    cv::imshow( "Example 2-10", rgb );
    if( (char) cv::waitKey(33) >= 0 ) break;
  }

  return 0;
}