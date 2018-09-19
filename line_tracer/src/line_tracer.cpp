#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
//#include <opencv2/contrib/features2d.hpp>
#include <iostream>
#include <vector>
#include <ctime>
#include <cmath>

#include <stdio.h>
#include <time.h>
#include <sys/time.h>

#include "line_tracer/white_pixel.h"

using namespace std;

//constant of line color
const int WHITE = 255;
const int BLACK = 0;

//constant of image size
const int BINARY_IMAGE_WIDTH = 160;
const int BINARY_IMAGE_HEIGHT = 120;



class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;

  ros::NodeHandle wh_;
  ros::Publisher white_count_pub;
  line_tracer::white_pixel white_msg;
  int mode;

public:
  ImageConverter(int _mode, const char* _output_topicname)
    : it_(nh_), mode(_mode)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/input_data", 1,
      &ImageConverter::imageCb, this);

    image_pub_ = it_.advertise(_output_topicname, 1000);

    white_count_pub = wh_.advertise<line_tracer::white_pixel>("white_count", 1000);

    //cv::namedWindow(OPENCV_WINDOW);
  }

  ~ImageConverter()
  {
    //cv::destroyWindow(OPENCV_WINDOW);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    //ROS msg convert to OPENCV MAT
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    cv::Mat originalImg(cv_ptr->image);

    cv::Mat gray_img;
    cvtColor(cv_ptr->image, gray_img, CV_BGR2GRAY);

    cv::Mat binalized_img;
    threshold(gray_img, binalized_img, 160, WHITE, CV_THRESH_BINARY);

    int total_white_pix = 0;
    total_white_pix = detectLines(binalized_img.rows/2,binalized_img);

    cout<<"total_white_pix: "<<total_white_pix<<endl;

    cv::Mat debug_img;
    cvtColor(binalized_img, debug_img, CV_GRAY2BGR);
    line(debug_img, cv::Point(0,binalized_img.rows/2), cv::Point(160,binalized_img.rows/2), cv::Scalar(0,200,0), 3, CV_AA);

    white_msg.white_count=total_white_pix;
    static bool drawCircle = (mode==1);
    if(drawCircle) {
      image_pub_.publish(cv_bridge::CvImage(std_msgs::Header(), "bgr8", debug_img).toImageMsg());
      white_count_pub.publish(white_msg);

    } else {
      //image_pub_.publish(cv_bridge::CvImage(std_msgs::Header(), "mono8", debug_img).toImageMsg());
    }
  }

  int detectLines(int target_line,cv::Mat &image){
    int left_edge = 0;
    int count = 0;
    int white_pix_count = 0;

//Make line list.
for(int x=0; x < image.cols; x++){
  //cout << "img_cols =" << x << endl;
  int pix_val = image.at<uchar>(target_line,x);
  white_pix_count += pix_val == WHITE ? 1 : 0;

  if(pix_val == WHITE){
    if(x!=0 || image.at<uchar>(target_line,x-1) == BLACK)
      left_edge = x;
    count ++;
  }
  else {
    count = 0;
  }
}

return white_pix_count;
  }
};



int main(int argc, char** argv)
{
  ros::init(argc, argv, "line_tracer");
    if(argc!=3) {
    cout << "usage: rosrun line_tracer line_tracer OUTPUT_TOPICNAME MODE" << endl;
    cout << "MODE 0=FASTX(hardware) 1=drawFeature(Circle, software)" << endl;
    return -1;
  }
  int ic_mode  = atoi(argv[2]);
  cout << "MODE=" << ic_mode << endl;

  ImageConverter ic(ic_mode, argv[1]);
  ros::spin();
  return 0;
}
