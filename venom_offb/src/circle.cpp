#include "util.h" // venom
#include "Navigator.h"
#include <eigen_conversions/eigen_msg.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
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
    //t.rotate (Eigen::AngleAxisd (theta+M_PI/2.0, Eigen::Vector3d::UnitZ()));

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

  if (argc < 4) return 1;
  double radius = atof(argv[1]);
  double res = atof(argv[2]);
  double tol = atof(argv[3]);

  std::list<geometry_msgs::PoseStamped> circle = circle_traj(res,radius,1);

  nav = new venom::Navigator();
  nav->SetVerbose(true);
  nav->TakeOff(1.0);
  geometry_msgs::PoseStamped check_point;
  check_point.pose.position.x = check_point.pose.position.y = 0;
  check_point.pose.position.z = 1.0;
  //check_point.pose.orientation.w = check_point.pose.orientation.x 
  //= check_point.pose.orientation.y = check_point.pose.orientation.z = 0;

  ros::Duration d(0.05);

  while (ros::ok() && nav->GetStatus() != venom::NavigatorStatus::OFF) {
    int rc = venom::wait_key(0,1000,c);
    if (c == 'q' || rc < 0)
      break;

    nav->SetPoint(circle.front());
    if (nav->Error(circle.front()) < tol) {
      circle.push_back(circle.front());
      circle.pop_front();
      nav->SetPoint(circle.front());
    }
    d.sleep();
    ros::spinOnce();
  }

  ROS_INFO("quit and land");

  nav->SetPoint(check_point);
  while (ros::ok() && nav->Error(check_point) > 0.2)
    d.sleep();

  nav->Land();
  delete nav;
  return 0;
}
