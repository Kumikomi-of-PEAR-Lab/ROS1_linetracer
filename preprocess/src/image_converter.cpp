#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>
#include <vector>
#include <ctime>
#include <cmath>

#include <stdio.h>
#include <time.h>
#include <sys/time.h>


static const std::string OPENCV_WINDOW = "Image window";
using namespace std;


class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  int width;
  int height;

public:
  ImageConverter(int _width, int _height)
    : it_(nh_), width(_width), height(_height)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/image_raw", 1,
      &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("/input_data", 1000);

    //cv::namedWindow(OPENCV_WINDOW);
  }

  ~ImageConverter()
  {
    //cv::destroyWindow(OPENCV_WINDOW);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    //double d[7];
    //int t = 0;

    //d[t++] = get_dtime();

    //sleep(1);
    sleep(0.1);

    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGRA8);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    //d[t++] = get_dtime();

    cv::Mat resizedImg;
    cv::resize(cv_ptr->image, resizedImg, cv::Size(width,height));

    image_pub_.publish(cv_bridge::CvImage(std_msgs::Header(), "bgra8", resizedImg).toImageMsg());
    }

	/*double get_dtime(){
	  struct timeval tv;
	  gettimeofday(&tv, NULL);
	  return ((double)(tv.tv_sec) + (double)(tv.tv_usec) * 0.001 * 0.001);
	}*/
};



int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_converter");

  if(argc!=3) {
    cout << "usage: rosrun preprocess preprocess_node WIDTH HEIGHT" << endl;
    return -1;
  }
  int ic_width  = atoi(argv[1]);
  int ic_height = atoi(argv[2]);
  cout << "WIDTH =" << ic_width << endl;
  cout << "HEIGHT=" << ic_height << endl;

  ImageConverter ic(ic_width, ic_height);
  ros::spin();
  return 0;
}
