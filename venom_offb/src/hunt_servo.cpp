#include <ros/ros.h>
#include <std_msgs/Int32MultiArray.h>
#include <venom_offb/Navigator.h>
#include <venom_perception/Zed.h>
#include <eigen_conversions/eigen_msg.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include "util.h"
#include <string>

int px1=0,px2=0,py1=0,py2=0;
int center_x, center_y, tol_x, tol_y;
bool trigger = false;
static void bb_callback(std_msgs::Int32MultiArray::ConstPtr msg) {
  px1 = std::max(msg->data[0],0);
  py1 = std::max(msg->data[1],0);
  px2 = std::min(msg->data[2],center_x*2);
  py2 = std::min(msg->data[3],center_y*2);
  trigger = true;
}

venom::Navigator* nav;                                                          

void exit_handler(int s) {                                                      
  ROS_WARN("Force quitting...\n");
  //nav->Land();
  //delete nav;
  exit(1);
}

int main (int argc, char** argv) {
  ros::init(argc, argv, "visual_servo", ros::init_options::NoSigintHandler);
  // Retrieve input resolution of the YOLO detector
  int resolution_x, resolution_y;
  if (!ros::param::get("/venom/resolution/x", resolution_x)) {
    ROS_ERROR("Couldn't find rosparam /venom/resolution/x");
    return 1;
  } else if (!ros::param::get("/venom/resolution/y", resolution_y)) {
    ROS_ERROR("Couldn't find rosparam /venom/resotluion/y");
    return 1;
  }
  ROS_INFO_STREAM("Retrieve resotlution "<< resolution_x << ", " << resolution_y);
  tol_x = center_x / 12;
  tol_y = center_y / 12;

  // Visual servo parameters
  // max_theta : max. yaw angle turns at each step
  // max_z     : max. z-axis adjustment at each step
  // dist      : forward step size (set 0.0 on yawing scenario)
  // time_step : period of a time step. No lower than 0.1 sec. This is due to
  //             the update time of the Navigation thread.
  double max_theta = std::stod(argv[1]); // M_PI/3.0
  double max_z = std::stod(argv[2]);     // 0.3
  double dist = std::stod(argv[4]);  // 0.0
  double time_step = std::stod(argv[3]); // 0.1
  ROS_INFO_STREAM("Receive max_theta " << max_theta);
  ROS_INFO_STREAM("Receive max_z " << max_z);
  ROS_INFO_STREAM("Receive dist " << dist);
  ROS_INFO_STREAM("Receive time_step " << time_step);
  
  signal(SIGINT, exit_handler);
  ros::NodeHandle nh;
  ros::Subscriber bb_sub = nh.subscribe<std_msgs::Int32MultiArray>("/venom/bounding_box", 1, bb_callback);

  venom::Zed zed;
  zed.Enable(venom::PerceptionType::ODOM);

  //nav = new venom::Navigator();
  //nav->TakeOff(1.0);

  ros::Duration d(time_step);
  geometry_msgs::PoseStamped cmd;
  cmd.pose.position.x = 0.0;
  cmd.pose.position.y = 0.0;
  cmd.pose.position.z = 1.0;
  cmd.pose.orientation.x = 0.0;
  cmd.pose.orientation.y = 0.0;
  cmd.pose.orientation.z = 0.0;
  cmd.pose.orientation.w = 1.0;
  Eigen::Affine3d t;
  char c = 'x';
  int count = 0;
  bool ccw = true;

search_target:
  ROS_INFO("Searching target...");
  while (ros::ok()) {
    if (px1!=0 || px2!=0 || py1 != 0 || py2 != 0) break;
    venom::wait_key(0,1000,c);
    if (c == 'q')
      break;
    tf::poseMsgToEigen (cmd.pose, t);
    if (ccw)
      t.rotate (Eigen::AngleAxisd (M_PI/20.0, Eigen::Vector3d::UnitZ()));
    else
      t.rotate (Eigen::AngleAxisd (-M_PI/20.0, Eigen::Vector3d::UnitZ()));
    tf::poseEigenToMsg(t, cmd.pose);
    //nav->SetPoint(cmd);
    ros::spinOnce();
    d.sleep();
  }
  c = 'x';
  ROS_INFO("Begin z-axis following");
  while (ros::ok()) {
    venom::wait_key(0,1000,c);
    if (c == 'q')
      break;
    if (px1!=0 || px2!=0 || py1 != 0 || py2 != 0) {
      // Compute the center of bounding box
      int midx = (px1 + px2)/2, midy = (py1 + py2)/2;
      ROS_INFO_STREAM("target center (" << midx << ", " << midy << ")");

      double theta = 0.0, dz = 0.0;
      if (center_y - midy > tol_y ) {
        ROS_INFO("Go up");
        dz = max_z * std::min(static_cast<double>(center_y-midy)/static_cast<double>(center_y), 1.0);
        ROS_INFO_STREAM("dz = " << dz);
      } else if (midy - center_y > tol_y ) {
        ROS_INFO("Go down");
        dz = - max_z * std::min(static_cast<double>(midy-center_y)/static_cast<double>(center_y),1.0);
        ROS_INFO_STREAM("dz = " << dz);
      }
      if (midx - center_x > tol_x ) {
        ROS_INFO("Turn right");
        theta = - max_theta * std::min(static_cast<double>(midx-center_x)/static_cast<double>(center_x),1.0);
        ROS_INFO_STREAM("theta = " << theta);
        ccw = false;
      } else if (center_x - midx > tol_x ) {
        ROS_INFO("Turn left");
        theta = max_theta * std::min(static_cast<double>(center_x-midx)/static_cast<double>(center_x), 1.0);
        ROS_INFO_STREAM("theta = " << theta);
        ccw = true;
      }

      // Matrix tranformation
      cmd.pose.position = zed.GetPose().position;
      tf::poseMsgToEigen (cmd.pose, t);
      t.rotate (Eigen::AngleAxisd (theta, Eigen::Vector3d::UnitZ()));
      tf::poseEigenToMsg(t, cmd.pose);
      cmd.pose.position.x += dist * cos(theta);
      cmd.pose.position.y += dist * sin(theta);
      cmd.pose.position.z += dz;
      //nav->SetPoint(cmd);

      px1 = py1 = px2 = py2 = 0; // clear buffered values
      count = 10;
    } else {
      if (--count <= 0) goto search_target;
    }
    d.sleep();
    ros::spinOnce();
  }
  //nav->Land(0.8);
  return 0;
}
