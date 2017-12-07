#include <iostream>
#include <ros/ros.h>
#include <std_msgs/Char.h>
#include "util.h"
int main(int argc, char **argv) {
  ros::init(argc, argv, "AI");
  ros::NodeHandle nh;
  ros::Publisher pub = nh.advertise<std_msgs::Char>("/venom/high_level", 100);
  bool exit_ = false;
  char c;
  std_msgs::Char c_msg;
  while ( !exit_ ) {
    c = ' ';
    int res = venom::wait_key(1,0,c);
    if (res < 0) {
      std::cout << "error: select fail\n";
      break;
    }

    switch (c) {
      case 'a':
	std::cout << "Go left\n";
	break;
      case 'w':
      case 'W':
	std::cout << "Go up\n";
	break;
      case 'd':
      case 'D':
	std::cout << "Go right\n";
	break;
      case 's':
      case 'S':
	std::cout << "Go down\n";
	break;
      case 'q':
      case 'Q':
	std::cout << "Exit\n";
	exit_ = true;
	break;
    }
    c_msg.data = c;
    pub.publish(c_msg);
    ros::spinOnce();
  }

  return 0;
}
