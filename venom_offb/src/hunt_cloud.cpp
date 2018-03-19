#include "util.h" // venom
#include <venom_offb/Navigator.h>
#include <eigen_conversions/eigen_msg.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
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
      nav->GotoYour( target_pos);
      c = 'x';
    }

    d.sleep();
    ros::spinOnce();
  }


  nav->TakeOff(1.0); // TODO: this is bad... try to redesign the class pattern
  nav->SetTolerence(0.1);
  ROS_INFO("Back to 1 meter high");
  while (ros::ok() && nav->GetStatus() != venom::NavigatorStatus::IDLE) {
    d.sleep();
    ros::spinOnce();
  }
  ROS_INFO("Landing...");

  nav->Land();
  delete nav;
  return 0;
}
