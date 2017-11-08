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



geometry_msgs::PoseWithCovarianceStamped currentPose;
void odom_cb(const nav_msgs::Odometry& msg){
	currentPose.header = msg.header;
	currentPose.pose = msg.pose;
}


ros::Publisher cloud_pub;
sensor_msgs::PointCloud2 current_cloud;

void 
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
	// Container for original and filtered data
	pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
	pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
	pcl::PCLPointCloud2 cloud_filtered;

	// Convert to PCL data type
	pcl_conversions::toPCL(*cloud_msg, *cloud);

	// Save pcd
	pcl::io::savePCDFile ("cloud.pcd", *cloud);

	// Convert to ROS data type
	sensor_msgs::PointCloud2 output;
	pcl_conversions::fromPCL(cloud_filtered, output);

	// Publish the data
	cloud_pub.publish(output);

}

void 
cloud_voxgrid_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
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
	pcl::io::savePCDFile ("voxgrid.pcd", cloud_filtered);

	// Convert to ROS data type
	sensor_msgs::PointCloud2 output;
	pcl_conversions::fromPCL(cloud_filtered, output);

	// Publish the data
	cloud_pub.publish(output);

}

// TODO: Arrow Marker
ros::Publisher marker_pub;
visualization_msgs::Marker marker;
void setBack()
{
	marker.header.frame_id = "map";
	marker.header.stamp = ros::Time();
	marker.ns = "/venom";
	marker.id = 0;
	marker.type = visualization_msgs::Marker::ARROW;
	marker.action = visualization_msgs::Marker::ADD;
	marker.pose = currentPose.pose.pose;
	marker.pose.orientation.x *= -1.0; // Reverse direction
	marker.pose.orientation.y *= -1.0; // Reverse direction
	marker.pose.orientation.z *= -1.0; // Reverse direction
	//marker.pose.orientation.w *= -1.0; // Reverse direction
	marker.scale.x = 1.0;
	marker.scale.y = 0.1;
	marker.scale.z = 0.1;
	marker.color.a = 1.0; // Don't forget to set the alpha!
	marker.color.r = 1.0f;
	marker.color.g = 0.0f;
	marker.color.b = 0.0f;
	marker.lifetime = ros::Duration();
	marker_pub.publish( marker);
}

void calcDirection()
{
	marker.header.frame_id = "map";
	marker.header.stamp = ros::Time();
	marker.ns = "/venom";
	marker.id = 0;
	marker.type = visualization_msgs::Marker::ARROW;
	marker.action = visualization_msgs::Marker::ADD;
	//marker.pose.position.x = 0;
	//marker.pose.position.y = 0;
	//marker.pose.position.z = 0;
	//marker.pose.orientation.x = 0.0;
	//marker.pose.orientation.y = 0.0;
	//marker.pose.orientation.z = 0.0;
	//marker.pose.orientation.w = 0.0;
	marker.pose = currentPose.pose.pose;
	marker.scale.x = 1.0;
	marker.scale.y = 0.1;
	marker.scale.z = 0.1;
	marker.color.a = 1.0; // Don't forget to set the alpha!
	marker.color.r = 0.0f;
	marker.color.g = 1.0f;
	marker.color.b = 0.0f;
	marker.lifetime = ros::Duration();
	marker_pub.publish( marker);
}

void direction_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
	// Container for original and filtered data
	pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2;
	pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
	pcl::PCLPointCloud2 cloud_filtered;

	// Convert to PCL data type
	pcl_conversions::toPCL(*cloud_msg, *cloud);

	// Perform the actual filtering
	pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
	sor.setInputCloud (cloudPtr);
	sor.setLeafSize (0.05,0.05,0.05);
	sor.filter (cloud_filtered);
	std::cout << "Number of point: " << cloud_filtered.data.size() << std::endl;

	if (cloud_filtered.data.size() < 250000 )
		setBack();
	else
		calcDirection();
}


int main (int argc, char** argv)
{
	ros::init(argc, argv, "cloud_reader");
	ros::NodeHandle nh;

	ros::Subscriber cloud_sub = nh.subscribe("/zed/point_cloud/cloud_registered", 1, direction_cb);
	cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/venom/point_cloud",1);

	ros::Subscriber odom_sub = nh.subscribe("/zed/odom", 10, odom_cb);
	// TODO: Arrow Marker
	// Publish direction
	marker_pub = nh.advertise<visualization_msgs::Marker>( "/venom/direction", 10 );

	ros::Rate rate(1);
	while (ros::ok())
	{
		ros::spinOnce();
		rate.sleep();
	}
	return 0;
}
