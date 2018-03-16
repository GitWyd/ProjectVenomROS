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
#include <venom_perception/Zed.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <std_msgs/Int32MultiArray.h>

int a1=0,a2=0,b1=0,b2=0;
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

// Callback function from YOLO
// Update bounding box indices
static void bb_callback(std_msgs::Int32MultiArray::ConstPtr msg) {
  a1 = msg->data[0];
  b1 = msg->data[1];
  a2 = msg->data[2];
  b1 = msg->data[3];
}

int main (int argc, char** argv) {
  ros::init(argc, argv, "detect_drone");
  ros::NodeHandle nh;
  ros::Publisher target_pub = nh.advertise<geometry_msgs::Point>("/venom/target_pos", 10);
  ros::Subscriber bb_sub = nh.subscribe<std_msgs::Int32MultiArray>("/venom/bounding_box", 1, bb_callback);
  venom::Zed zed;
  zed.Enable(venom::PerceptionType::RGB);
  zed.Enable(venom::PerceptionType::CLOUD);
  ros::Rate rate(10);
  while (ros::ok()) {
    cv::Mat rgb = zed.GetRGB();
    if (a1 == 0 && a2 == 0 && b1 == 0 && b2 == 0) {
      ROS_DEBUG("Nothing detected");
      ros::spinOnce();
      rate.sleep();
      continue;
    }
    ROS_INFO_STREAM("Detect bounding box: ("<< a1 << "," << b1 << ") to ("
                    << a2 << "," << b2 << ")");
    cv::Point pt1(a1, b1);
    cv::Point pt2(a2, b2);
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

    }
    ros::spinOnce();
    rate.sleep();
  }
  return 0;
}
