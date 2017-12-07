#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>


void depth_cb(const sensor_msgs::ImageConstPtr& msg){
  std::cout<<"Image dimension: "<<msg->width<<", "<<msg->height << std::endl;
  std::cout<<"Encoding: "<<msg->encoding<<std::endl;
  cv::Mat img(msg->height, msg->width,cv::CV_8U,msg->data,msg->step);
}

int main(int argc, char** argv){
  ros::init(argc, argv, "obstacle_detector");
  ros::NodeHandle nh;
  ros::Subscriber depth_sub = nh.subscribe("/zed/depth/depth_registered",
                                           1, depth_cb);
  ros::spin();
  return 0;
}
