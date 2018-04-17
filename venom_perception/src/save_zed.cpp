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
#include "Perceiver.h"
#include <venom_perception/Zed.h>

int main (int argc, char** argv) {
  ros::init(argc, argv, "perception_node");
  venom::Perceiver pcv;
  venom::Zed zed;
  zed.Enable(venom::PerceptionType::RGB);
  cv::Mat rgb;
  ros::Rate rate(5);
  int count = 0;
  std::string pcl_prefix = "cloud_";
  std::string pcl_postfix = ".pcd";
  std::string pose_prefix = "pose_";
  std::string pose_postfix = ".txt";
  std::string cv_prefix = "rgb_";
  std::string cv_postfix = ".png";
  
  while (ros::ok())
  {
    pcv.SavePointCloud(pcl_prefix+std::to_string(count)+pcl_postfix);
    pcv.SavePose(pose_prefix+std::to_string(count)+pose_postfix);
    rgb = zed.GetRGB();
    cv::imwrite(cv_prefix+std::to_string(count)+cv_postfix, rgb);
    ros::spinOnce();
    rate.sleep();
    count += 5;
  }
  return 0;
}
