#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_datatypes.h>
#include <list>
#include <vector>
#include <iostream>
#include <string>
#include <fstream>


geometry_msgs::PoseWithCovarianceStamped currentPose;
void odom_callback(const nav_msgs::Odometry& msg){
	currentPose.header = msg.header;
	currentPose.pose = msg.pose;
}

int main(int argc, char** argv){
	ros::init(argc,argv,"visual_pose");
	ros::NodeHandle n;
	ros::Subscriber odom = n.subscribe("/zed/odom",5,odom_callback);
	ros::Publisher pose_pub = 
		n.advertise<geometry_msgs::PoseWithCovarianceStamped>
		("mavros/vision_pose/pose_cov", 10);

	int hz;
	if (argc > 1)
		hz = atoi(argv[1]);
	else
		hz = 1;
	ros::Rate rate(hz);

	while(ros::ok()){
		ros::spinOnce();
		ROS_INFO("Position = [%4.2f, %4.2f, %4.2f]",
				currentPose.pose.pose.position.x,
				currentPose.pose.pose.position.y,
				currentPose.pose.pose.position.z);
		ROS_INFO("Quaternion = [%4.2f, %4.2f, %4.2f, %4.2f]",
				currentPose.pose.pose.orientation.x,
				currentPose.pose.pose.orientation.y,
				currentPose.pose.pose.orientation.z,
				currentPose.pose.pose.orientation.w);
		rate.sleep();
		pose_pub.publish(currentPose);
	}
	return 0;
}
