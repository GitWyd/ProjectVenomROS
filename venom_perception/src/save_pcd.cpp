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

int count = 0;
bool save_odom = false;
bool save_pcd = false;

void odom_cb(const nav_msgs::Odometry& msg){
  if (!save_odom)
    return;
  geometry_msgs::Point p = msg.pose.pose.position;
  geometry_msgs::Quaternion o = msg.pose.pose.orientation;
  std::string filename = "pose"+std::to_string(count)+".txt";
  std::cout << "Saving file " << filename << "... ";
  
  std::ofstream file (filename);
    if (file.is_open()) {
    file << p.x <<  ", " << p.y <<  ", " << p.z << std::endl;
    file << o.x <<  ", " << o.y <<  ", " << o.z << ", " << o.w << std::endl;
    file.close();
    save_odom = false;
    std::cout << "done" << std::endl;
  } else std::cout << "Unable to open file";
}

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg) {
  if (!save_pcd)
    return;
  // Container for original and filtered data
  pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
  pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
  pcl::PCLPointCloud2 cloud_filtered;
  
  // Convert to PCL data type
  pcl_conversions::toPCL(*cloud_msg, *cloud);
  
  // Perform the actual filtering
  pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
  sor.setInputCloud (cloudPtr);
  sor.setLeafSize (0.01,0.01,0.01);
  sor.filter (cloud_filtered);
  
  // Save pcd
  std::string filename = "cloud"+std::to_string(count)+".pcd";
  std::cout << "Saving file " << filename << "... ";
  pcl::io::savePCDFile (filename, cloud_filtered);
  save_pcd = false;
  std::cout << "done" << std::endl;
}

int main (int argc, char** argv) {
  ros::init(argc, argv, "cloud_reader");
  ros::NodeHandle nh;

  ros::Subscriber cloud_sub = nh.subscribe("/zed/point_cloud/cloud_registered", 1, cloud_cb);

  ros::Subscriber odom_sub = nh.subscribe("/zed/odom", 10, odom_cb);

  std::cout << "Press ENTER to capture sample\n";
  ros::Rate rate(1);
  while (ros::ok()) {
    std::cin.get();
    save_odom = true;
    save_pcd = true;
    ros::spinOnce();
    rate.sleep();
    count++;
  }
  return 0;
}
