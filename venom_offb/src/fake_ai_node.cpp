#include <iostream>
#include <ros/ros.h>
#include <std_msgs/Char.h>
#include "util.h"
#include "Navigator.h"
#include <signal.h>

venom::Navigator* nav;

void exit_handler(int s) {
  ROS_WARN("Force quitting...\n");
  nav->Land();
  delete nav;
  exit(1);
}

char cmd;
void cmd_callback(const std_msgs::Char::ConstPtr& msg) {
  cmd = msg->data;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "fakeAI", ros::init_options::NoSigintHandler);
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("/venom/high_level", 10, cmd_callback);

  signal(SIGINT, exit_handler);

  geometry_msgs::PoseStamped cmd_pose;
  cmd_pose.pose.position.x = cmd_pose.pose.position.y = 0;
  cmd_pose.pose.position.z = 2.0;
  cmd_pose.pose.orientation.x = cmd_pose.pose.orientation.y = cmd_pose.pose.orientation.z
    = cmd_pose.pose.orientation.w = 0;

  nav = new venom::Navigator();
  nav->TakeOff(2.0);

  if (nav->GetStatus() == venom::NavigatorStatus::OFF) {
    ROS_ERROR("Takeoff failed");
    return 1;
  }

  bool exit_ = false;
  double dx = 0.0;
  double dy = 0.0;
  double dz = 0.0;
  ros::Duration d(0.1);

  while ( ros::ok() && !exit_ ) {
    char c = ' ';
    int res = venom::wait_key(1,0,c);
    if (res < 0) {
      std::cout << "error: select fail\n";
      break;
    }

    switch (c) {
      case 'a':
      case 'A':
	std::cout << "Go left\n";
	dy = 0.5;
	dx = dz = 0.0;
	break;
      case 'w':
      case 'W':
	std::cout << "Go forward\n";
	dx = 0.5;
	dy = dz = 0.0;
	break;
      case 'd':
      case 'D':
	std::cout << "Go right\n";
	dy = -0.5;
	dx = dz = 0.0;
	break;
      case 's':
      case 'S':
	std::cout << "Go backward\n";
	dx = -0.5;
	dy = dz = 0.0;
	break;
      case 'i':
      case 'I':
	std::cout << "Go up\n";
	dz = 0.5;
	dx = dy = 0.0;
	break;
      case 'k':
      case 'K':
	std::cout << "Go down\n";
	dz = -0.5;
	dx = dy = 0.0;
	break;
      case 'q':
      case 'Q':
	std::cout << "Exit\n";
	exit_ = true;
	break;
    }

    if (nav->GetStatus() == venom::NavigatorStatus::IDLE
	&& (dx != 0 || dy != 0 || dz != 0)) {
      std::cout << "Update command pose: ";
      cmd_pose.pose.position.x += dx;
      std::cout << cmd_pose.pose.position.x << ", ";
      cmd_pose.pose.position.y += dy;
      std::cout << cmd_pose.pose.position.y << ", ";
      cmd_pose.pose.position.z += dz;
      std::cout << cmd_pose.pose.position.z << std::endl;
      nav->SetPoint(cmd_pose);
      dx = dy = dz = 0.0;
    }

    d.sleep();
    ros::spinOnce();
  }

  nav->Land();

  return 0;
}
