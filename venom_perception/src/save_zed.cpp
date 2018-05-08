/*******************************************************************************
* save_zed.cpp
* 
* This program saves RGB image, point cloud, and pose information using 
* venom::Zed.
*
* Author: Yan-Song Chen
* Date  : May 8, 2018
*******************************************************************************/
#include <string>                         // std
#include <sstream>
#include <fstream>
#include <thread>
#include <opencv2/opencv.hpp>             // OpenCV
#include <opencv2/highgui/highgui.hpp>
#include <pcl/point_cloud.h>              // PCL
#include <pcl/io/pcd_io.h>
#include <ros/ros.h>                      // ROS
#include <geometry_msgs/Pose.h>           // Venom
#include <venom_perception/Zed.h>

// Cross-thread variables
bool save_request = false;                // Trigger for data-saving procedure
bool save_active = true;
int count = 0;

cv::Mat rgb;
geometry_msgs::Pose p;
pcl::PointCloud<pcl::PointXYZRGB> cloud;
std::ostringstream pout, qout;            // Shared string buffer

std::string pcl_prefix = "cloud_";
std::string pcl_postfix = ".pcd";
std::string pose_prefix = "pose_";
std::string pose_postfix = ".txt";
std::string cv_prefix = "rgb_";
std::string cv_postfix = ".png";

// Data-saving thread
static void save_data() {
  while (save_active) {
    if (save_request) {
      cv::imwrite(cv_prefix + std::to_string(count) + cv_postfix, rgb);
      pcl::io::savePCDFileASCII(pcl_prefix + std::to_string(count) +
                                pcl_postfix, cloud);

      std::ofstream txt;
      txt.open(pose_prefix+std::to_string(count)+pose_postfix);
      txt << pout.str() << '\n';
      txt << qout.str();
      txt.close();

      count++;
      ROS_INFO_STREAM("Save data set " << count);
    }
    save_request = false;
  }
}

int main (int argc, char** argv) {
  ros::init(argc, argv, "perception_node");
  std::thread save_thread(save_data);

  // Note that each perception type must be enabled before request
  venom::Zed zed;
  zed.Enable(venom::PerceptionType::RGB);
  zed.Enable(venom::PerceptionType::ODOM);
  zed.Enable(venom::PerceptionType::CLOUD);
  
  while (ros::ok())
  {
    rgb = zed.GetRGB();
    p = zed.GetPose();
    cloud = zed.GetCloud();

    // Print help on copied image
    cv::Mat copy_img = rgb.clone();
    cv::putText(copy_img, std::string("Press: <q> exit <s> save"),
                cv::Point(rgb.cols*0.55, rgb.rows*0.8),
                cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255,255,255), 0.06);

    // Print position on copied image
    pout.str("");
    pout << std::setprecision(2) << "Position:" << p.position.x << ", " 
         << p.position.y << ", " << p.position.z;
    cv::putText(copy_img,pout.str(), cv::Point(rgb.cols*0.55, rgb.rows*0.85),
                cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255,255,255), 0.06);

    // Print orientation on copied image
    qout.str("");
    qout << std::setprecision(2) << "Orientation:" << p.orientation.x << ", "
         << p.orientation.y << ", " << p.orientation.z;
    cv::putText(copy_img,qout.str(), cv::Point(rgb.cols*0.55, rgb.rows*0.9),
                cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(255,255,255), 0.06);

    // Display and get key command
    cv::imshow("display", copy_img);
    char c = cv::waitKey(3);
    if (c == 'q' || c == 'Q') break;
    else if (c == 's' || c == 'S' ) save_request = true;
 
    ros::spinOnce();
  }
  save_active = false; // terminate the data-saving thread
  save_thread.join();
  cv::destroyWindow("display");
  return 0;
}
