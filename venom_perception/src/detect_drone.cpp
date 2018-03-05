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

std::pair<int,int> begin, end;
bool release = true;
void mouse_callback(int event, int x, int y, int flags, void* userdata) {
  if  ( event == cv::EVENT_LBUTTONDOWN ) {
    std::cout << "Left button of the mouse is clicked - position (" << x << ", " << y << ")" << std::endl;
  } else if  ( event == cv::EVENT_RBUTTONDOWN ) {
    std::cout << "Right button of the mouse is clicked - position (" << x << ", " << y << ")" << std::endl;
  } else if  ( event == cv::EVENT_MBUTTONDOWN ) {
    std::cout << "Middle button of the mouse is clicked - position (" << x << ", " << y << ")" << std::endl;
  } else if ( event == cv::EVENT_MOUSEMOVE ) {
    std::cout << "Mouse move over the window - position (" << x << ", " << y << ")" << std::endl;
  }
}

int main (int argc, char** argv) {
  ros::init(argc, argv, "detect_drone");
  venom::Perceiver pcv;
  ros::Rate rate(10);
  cv::namedWindow( "drone_fpv", cv::WINDOW_AUTOSIZE );
  cv::setMouseCallback("drone_fpv", mouse_callback, NULL);
  char key;
  while (ros::ok() && key != 'q') {
    if (pcv.RGBReady()) {
      cv::Mat rgb = pcv.GetRGBImage();
      cv::imshow("drone_fpv",rgb);
      key = cv::waitKey(50);
    }
    ros::spinOnce();
    rate.sleep();
  }
  cv::destroyAllWindows();
  return 0;
}
