#include <ros/ros.h>                                                            
#include <geometry_msgs/Pose.h>                                                 
#include <geometry_msgs/PoseWithCovarianceStamped.h>                            
#include <nav_msgs/Odometry.h>                                                  
#include <pcl_ros/point_cloud.h>                                                
#include <pcl/point_types.h>                                                    
#include <std_msgs/Header.h>
                                                                                
#include <pcl/io/pcd_io.h>                                                      
#include <sensor_msgs/PointCloud.h>                                             
#include <sensor_msgs/PointCloud2.h>                                            
#include <pcl_conversions/pcl_conversions.h>                                    
#include <pcl/point_cloud.h>                                                    
#include <pcl/point_types.h>                                                    
#include <pcl/filters/voxel_grid.h>                                             
                                                                                
#include <cv_bridge/cv_bridge.h>                                                
#include <opencv2/opencv.hpp>                                                   
#include <opencv2/core/core.hpp>                                                
#include <opencv2/highgui/highgui.hpp>                                          
#include <opencv2/imgproc/imgproc.hpp>                                          
                                                                                
#include <iostream>

#ifndef VENOM_PERCEPTION_ZED_H
#define VENOM_PERCEPTION_ZED_H
namespace venom {
enum PerceptionType {
  ODOM = 0,
  DEPTH = 1,
  RGB = 2,
  CLOUD = 3,
};

class Zed {
public:
  Zed() {
    nh_ = ros::NodeHandle();
  }
  void Enable(PerceptionType type);
  void Disable(PerceptionType type);
  geometry_msgs::Pose GetPose();
  cv::Mat GetRGB();
  cv::Mat GetDepth();
  void SetROI(int x1, int x2, int y1, int y2);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr GetCloud();
  std_msgs::Header GetHeader();

private:
  ros::Subscriber odom_sub_;  // odometry
  geometry_msgs::PoseWithCovarianceStamped pose_;
  void OdometryCallback(const nav_msgs::Odometry& msg);

  ros::Subscriber depth_sub_; // depth image
  cv_bridge::CvImagePtr depth_ptr_;
  void DepthCallback(const sensor_msgs::ImageConstPtr& msg);

  ros::Subscriber rgb_sub_;   // rgb image
  cv_bridge::CvImagePtr rgb_ptr_;
  void RGBCallback(const sensor_msgs::ImageConstPtr& msg);

  ros::Subscriber cloud_sub_;
  std_msgs::Header header_;
  pcl::PointCloud<pcl::PointXYZRGB> cloud_; // point cloud TODO: use pcl::PointCloudXYZRGB instead
  std::pair<int,int> roi_x_{0,0};
  std::pair<int,int> roi_y_{0,0};
  void CloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg);

  bool verbose_ = false;
  ros::NodeHandle nh_;
};

void Zed::Enable(PerceptionType type) {
  ros::NodeHandle nh;
  ros::Duration time(0.001);
  switch (type) {
    case PerceptionType::ODOM:
      odom_sub_ = nh.subscribe("/zed/odom", 10, &Zed::OdometryCallback, this);
      break;
    case PerceptionType::DEPTH:
      depth_sub_ = nh.subscribe("/zed/depth/depth_registered", 10, &Zed::DepthCallback, this);
      while (depth_ptr_ == NULL) {
        ros::spinOnce();
        time.sleep();
      }
      break;
    case PerceptionType::RGB:
      rgb_sub_ = nh.subscribe("/zed/rgb/image_rect_color",10,&Zed::RGBCallback, this);
      while (rgb_ptr_ == NULL) {
        ros::spinOnce();
        time.sleep();
      }
      break;
    case PerceptionType::CLOUD:
      cloud_sub_ = nh.subscribe("/zed/point_cloud/cloud_registered", 10, &Zed::CloudCallback, this);
      break;
    default:
      ROS_WARN("Enable failed - invalid type");
  }
}

void Zed::Disable(PerceptionType type) {
  switch (type) {
    case PerceptionType::ODOM:
      odom_sub_.shutdown();
      break;
    case PerceptionType::DEPTH:
      depth_sub_.shutdown();
      break;
    case PerceptionType::RGB:
      rgb_sub_.shutdown();
      break;
    case PerceptionType::CLOUD:
      cloud_sub_.shutdown();
      break;
    default:
      ROS_WARN("Disable failed - invalid type");
  }
}

void Zed::OdometryCallback(const nav_msgs::Odometry& msg) {                        
  pose_.header = msg.header;
  pose_.pose = msg.pose;
}

void Zed::DepthCallback(const sensor_msgs::ImageConstPtr& msg) {              
  try { 
    depth_ptr_ = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
  }                                                                           
} 

void Zed::RGBCallback(const sensor_msgs::ImageConstPtr& msg) {
    try {                       
      rgb_ptr_ = cv_bridge::toCvCopy(msg);
      if (verbose_) {
        ROS_DEBUG_STREAM("image size: " << rgb_ptr_->image.rows << ", " << rgb_ptr_->image.cols);
      }
    } catch (cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
    }
}

void Zed::CloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg) {
  header_ = cloud_msg->header;
  pcl::PointCloud<pcl::PointXYZRGB> pcl_cloud_raw;
  pcl::fromROSMsg(*cloud_msg,pcl_cloud_raw);
  cloud_.clear();
  for (int i = roi_x_.first; i < roi_x_.second; i++) {
    for (int j = roi_y_.first; j < roi_y_.second; j++) {
      cloud_.push_back(pcl_cloud_raw.at(i,j));
    }
  }
}

cv::Mat Zed::GetRGB() {
  return cv::Mat(rgb_ptr_->image);
}

cv::Mat Zed::GetDepth() {
  return cv::Mat(depth_ptr_->image);
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr Zed::GetCloud() {
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr res(new pcl::PointCloud<pcl::PointXYZRGB>(cloud_));
  return res;
}

std_msgs::Header Zed::GetHeader() {
  return header_;
}

void Zed::SetROI(int x1, int x2, int y1, int y2) {
  roi_x_ = {x1,x2};
  roi_y_ = {y1,y2};
}

}
#endif
