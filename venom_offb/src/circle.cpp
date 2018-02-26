#include "util.h" // venom
#include "Navigator.h"
#include <eigen_conversions/eigen_msg.h>
#include <cmath> // std
#include <list>
#include <iostream>
#include <signal.h>

venom::Navigator* nav;

void exit_handler(int s) {
  ROS_WARN("Force quitting...\n");
  nav->Land();
  delete nav;
  exit(1);
}

std::list<geometry_msgs::PoseStamped> circle_traj(double res, double r, double h) {
  std::list<geometry_msgs::PoseStamped> traj;
  double tick = 2.0*M_PI/res, theta = 0.0;
  for (double i = 0; i < res; i++) {
    Eigen::Affine3d t = Eigen::Affine3d::Identity();
    t.translation() << r*cos(theta), r*sin(theta), h;
    t.rotate (Eigen::AngleAxisd (theta+M_PI/2.0, Eigen::Vector3d::UnitZ()));

    geometry_msgs::PoseStamped cmd;
    tf::poseEigenToMsg(t, cmd.pose);
    traj.push_back(cmd);
    theta += tick;
  }
  return traj;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "Navigator", ros::init_options::NoSigintHandler);
  signal(SIGINT, exit_handler);
  char c = ' ';
  int rc;

  std::list<geometry_msgs::PoseStamped> circle = circle_traj(8,1.5,1);

  nav = new venom::Navigator();
  nav->TakeOff(1.0);
  ros::Duration d(0.5);
  nav->SetTolerence(0.3);
  while (ros::ok() && nav->GetStatus() != venom::NavigatorStatus::OFF) {
    int rc = venom::wait_key(0,1000,c);
    if (c == 'q' || rc < 0)
      break;

    if (nav->GetStatus() == venom::NavigatorStatus::IDLE) {
      nav->SetPoint(circle.front());
      circle.push_back(circle.front());
      circle.pop_front();
    }
    d.sleep();
    ros::spinOnce();
  }

  ROS_INFO("quit and land");
  nav->Land();
  delete nav;
  return 0;
}
