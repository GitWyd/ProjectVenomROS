#include "util.h" // venom
#include <venom_offb/Navigator.h>
#include <eigen_conversions/eigen_msg.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <eigen_conversions/eigen_msg.h> // matrix manipulation
#include <cmath> // std
#include <list>
#include <iostream>
#include <signal.h>
#include <venom_perception/Zed.h>

venom::Navigator* nav;

void exit_handler(int s) {
  ROS_WARN("Force quitting...\n");
  nav->Land();
  delete nav;
  exit(1);
}

geometry_msgs::Point target_pos;
static void point_callback(geometry_msgs::Point::Ptr msg) {
  target_pos = *msg;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "hunt_drone", ros::init_options::NoSigintHandler);
  signal(SIGINT, exit_handler);
  char c = ' ';
  int rc;
  
  ros::NodeHandle nh;
  ros::Subscriber target_sub = nh.subscribe("/venom/target_pos",10,point_callback);

  venom::Zed zed;
  zed.Enable(venom::PerceptionType::ODOM);

  nav = new venom::Navigator();
  nav->SetVerbose(true);
  nav->TakeOff(1.0);

  ros::Duration d(0.05);
  while (ros::ok() && nav->GetStatus() != venom::NavigatorStatus::IDLE) {
    ros::spinOnce();
    d.sleep();
  }

  double tol = 0.3;
  nav->SetTolerence(tol);
  while (ros::ok() && nav->GetStatus() != venom::NavigatorStatus::OFF) {
    int rc = venom::wait_key(0,1000,c);
    if (c == 'q' || rc < 0)
      break;
    if (c == 'g' || c == 'G') {
      geometry_msgs::Pose curr = zed.GetPose();
      double theta = atan2(target_pos.y,target_pos.x);
      double dist = std::max(sqrt(target_pos.y*target_pos.y + target_pos.x*target_pos.x)-0.2, 0.0);
      Eigen::Affine3d t;
      tf::poseMsgToEigen (curr, t);
      t.translation() << curr.position.x + dist, curr.position.y, curr.position.z+target_pos.z;
      t.rotate (Eigen::AngleAxisd (theta, Eigen::Vector3d::UnitZ()));

      geometry_msgs::PoseStamped cmd;
      tf::poseEigenToMsg(t, cmd.pose);
      nav->SetPoint(cmd);
      c = 'x';
    }

    d.sleep();
    ros::spinOnce();
  }


  nav->Land(0.8);
  delete nav;
  return 0;
}
