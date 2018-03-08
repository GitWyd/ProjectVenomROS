#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include "Zed.h"
#include <pcl/visualization/pcl_visualizer.h>

int a1,a2,b1,b2;
bool pressed = false;
bool trigger = false;
void mouse_callback(int event, int x, int y, int flags, void* userdata) {
  if ( !pressed && event == cv::EVENT_LBUTTONDOWN) {
    a1 = x;
    b1 = y;
    ROS_INFO_STREAM("begin = " << a1 << ", " << b1);
    pressed = true;
  } else if (pressed && event == cv::EVENT_LBUTTONUP) {
    a2 = x;
    b2 = y;
    ROS_INFO_STREAM("end = " << b2 << ", " << b2);
    pressed = false;
    trigger = true;
  }
}

int main (int argc, char** argv) {
  ros::init(argc, argv, "detect_drone");
  ros::NodeHandle nh;
  ros::Publisher cloud_pub_ = nh.advertise<sensor_msgs::PointCloud2>("/venom/enemy_drone", 1);
  venom::Zed zed;
  zed.Enable(venom::PerceptionType::RGB);
  zed.Enable(venom::PerceptionType::CLOUD);
  ros::Rate rate(10);
  cv::namedWindow( "drone_fpv", cv::WINDOW_AUTOSIZE );
  cv::setMouseCallback("drone_fpv", mouse_callback, NULL);
  char key;
  pcl::visualization::PCLVisualizer viewer("target");
  while (ros::ok() && key != 'q') {
    cv::Mat rgb = zed.GetRGB();
    cv::Point pt1(a1, b1);
    cv::Point pt2(a2, b2);
    cv::rectangle(rgb, pt1, pt2, cv::Scalar(0, 0, 255));
    
    cv::imshow("drone_fpv",rgb);
    key = cv::waitKey(50);
    if (trigger) {
      zed.SetROI(a1,a2,b1,b2);
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr enemy_cloud = zed.GetCloud();
      viewer.removePointCloud("target");
      viewer.addPointCloud(enemy_cloud, std::string("target"));
    }
    viewer.spinOnce();
    //sensor_msgs::PointCloud2 message;
    //message.header = zed.GetHeader();
    //pcl::toROSMsg(enemy_cloud,message);
    //cloud_pub_.publish(message);
    ros::spinOnce();
    rate.sleep();
  }
  cv::destroyAllWindows();
  return 0;
}
