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
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>

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
  ros::Publisher target_pub = nh.advertise<geometry_msgs::Point>("/venom/target_pos", 10);
  venom::Zed zed;
  zed.Enable(venom::PerceptionType::RGB);
  zed.Enable(venom::PerceptionType::CLOUD);
  ros::Rate rate(10);
  cv::namedWindow( "drone_fpv", cv::WINDOW_AUTOSIZE );
  cv::setMouseCallback("drone_fpv", mouse_callback, NULL);
  char key;
  //pcl::visualization::PCLVisualizer viewer("target");
  while (ros::ok() && key != 'q') {
    cv::Mat rgb = zed.GetRGB();
    cv::Point pt1(a1, b1);
    cv::Point pt2(a2, b2);
    cv::rectangle(rgb, pt1, pt2, cv::Scalar(0, 0, 255));
    
    cv::imshow("drone_fpv",rgb);
    key = cv::waitKey(50);
    if (trigger) {
      zed.SetROI(a1,a2,b1,b2);
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr roi = zed.GetCloud();

      pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZRGB>());
      pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
      sor.setInputCloud (roi);
      sor.setMeanK (50);
      sor.setStddevMulThresh (1.0);
      sor.filter (*filtered);

      if (!filtered->empty()) {
      pcl::PointXYZ centroid;
      pcl::computeCentroid(*filtered,centroid);
      geometry_msgs::Point target_pos;

      target_pos.x = centroid.x;
      target_pos.y = centroid.y;
      target_pos.z = centroid.z;
      target_pub.publish(target_pos);
      }

      //viewer.removePointCloud("target");
      //viewer.addPointCloud(filtered, std::string("target"));
    }
    //viewer.spinOnce();
    ros::spinOnce();
    rate.sleep();
  }
  cv::destroyAllWindows();
  return 0;
}
