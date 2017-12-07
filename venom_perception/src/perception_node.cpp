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

namespace venom {
class Perceiver {
public:
  Perceiver() {
    ros::NodeHandle nh;
    cloud_sub_ = nh.subscribe("/zed/point_cloud/cloud_registered", 10, &Perceiver::CloudCallback, this);
    odom_sub_ = nh.subscribe("/zed/odom", 10, &Perceiver::OdometryCallback, this);
  }
  void SavePointCloud(const std::string& filename) {
    pcl::io::savePCDFile (filename, pcl_cloud_);
  }
  sensor_msgs::PointCloud2::ConstPtr GetRosPointCloud() {
    // Convert to ROS data type
    sensor_msgs::PointCloud2* cloud_ptr = new sensor_msgs::PointCloud2;
    pcl_conversions::fromPCL(pcl_cloud_, *cloud_ptr);
    sensor_msgs::PointCloud2::ConstPtr output(cloud_ptr);
    return output;
  }

private:
  pcl::PCLPointCloud2 pcl_cloud_; // Filtered and processed
  ros::Subscriber cloud_sub_;
  void CloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg) {
    // Container for original and filtered data
    pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
    pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);

    // Convert to PCL data type
    pcl_conversions::toPCL(*cloud_msg, *cloud);

    // Perform the actual filtering
    pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
    sor.setInputCloud (cloudPtr);
    sor.setLeafSize(0.1,0.1,0.1);
    sor.filter(pcl_cloud_); // TODO: what about the original data? Memory leak?
  }

  geometry_msgs::PoseWithCovarianceStamped pose_;
  ros::Subscriber odom_sub_;
  void OdometryCallback(const nav_msgs::Odometry& msg) {
    pose_.header = msg.header;
    pose_.pose = msg.pose;
    //std::string info = "Currrent Position: " + std::to_string(pose_.pose.pose.position.x) +
    //  std::to_string(pose_.pose.pose.position.y) + std::to_string(pose_.pose.pose.position.z);
    //ROS_INFO(info);
  }


  ros::Subscriber img_sub_;
  cv_bridge::CvImagePtr cv_ptr;
  void DepthImageCallback(const sensor_msgs::ImageConstPtr& msg) {
    try {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
    } catch (cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
  }
};
} // namespace venom


int main (int argc, char** argv) {
  ros::init(argc, argv, "perception_node");
  venom::Perceiver pcv;
  ros::Rate rate(1);
  while (ros::ok())
  {
	  ros::spinOnce();
	  rate.sleep();
  }
  return 0;
}
