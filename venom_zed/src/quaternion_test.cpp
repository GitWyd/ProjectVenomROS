#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <sl/Camera.hpp>

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <math.h>
#include <iostream>

static void set_marker(visualization_msgs::Marker& marker,
                       sl::float4 quaternion,
                       sl::float4 color) {
  marker.header.frame_id = "map";
  marker.header.stamp = ros::Time();
  marker.ns = "/venom";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::ARROW;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.orientation.x = quaternion[0];
  marker.pose.orientation.y = quaternion[1];
  marker.pose.orientation.z = quaternion[2];
  marker.pose.orientation.w = quaternion[3];
  marker.scale.x = 1.2f;
  marker.scale.y = 0.1f;
  marker.scale.z = 0.1f;
  marker.pose.position.x = 0;
  marker.pose.position.y = 0;
  marker.pose.position.z = 0;
  marker.color.r = color[0];
  marker.color.g = color[1];
  marker.color.b = color[2];
  marker.color.a = color[3];
  marker.lifetime = ros::Duration();
}

int main(int argc, char **argv) {

  ros::init(argc, argv, "sl_quaternion");
  ros::NodeHandle nh;
  ros::Publisher marker_pub; // Pose
  ros::Publisher ref_marker_pub; // Ref_Pose
  visualization_msgs::Marker marker;
  marker_pub = nh.advertise<visualization_msgs::Marker>( "/venom/navigation", 10 );
  ref_marker_pub = nh.advertise<visualization_msgs::Marker>( "/venom/pose", 10 );

  while (ros::ok() ) {
    // Reference pose
    sl::Orientation quaternion( sl::float4{0.5f, 0.3f, 0.0f, 0.2f});
    set_marker( marker, quaternion, sl::float4{1.0f, 1.0f, 1.0f, 0.5f} );
    ref_marker_pub.publish( marker);

    // Rotate about Z-axis
    sl::Matrix3f rotation = sl::Matrix3f::identity();
    float theta = M_PI/3.0f;
    rotation(0, 0) = std::cos(theta);
    rotation(0, 1) = -std::sin(theta);
    rotation(1, 0) = std::sin(theta);
    rotation(1, 1) = std::cos(theta);

    // Relative rotation is post-order
    sl::Rotation result = quaternion.getRotationMatrix() * rotation;
    std::cout << "---------------------\n"
              << result(0, 0) << ", " << result(0, 1) << ", " << result(0, 2) << std::endl
              << result(1, 0) << ", " << result(1, 1) << ", " << result(1, 2) << std::endl
              << result(2, 0) << ", " << result(2, 1) << ", " << result(2, 2) << std::endl;
    sl::float4 direction = result.getOrientation();
    set_marker( marker, direction, sl::float4{0.0f, 0.0f, 0.0f, 0.6f} );
    marker_pub.publish( marker);
    ros::spinOnce();
    ros::Duration(0.5).sleep();
  }
  return 0;
}
