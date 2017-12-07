#include <iostream>
#include "util.h"
#include "Navigator.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "Navigator");
  char c = ' ';
  int rc;
  geometry_msgs::PoseStamped pose;
  pose.pose.position.x = 0.0;
  pose.pose.position.y = 0.0;
  pose.pose.position.z = 3.0;


  venom::VenomNavigator nav;
  while (!nav.Ok() )
    ros::Duration(0.5).sleep();
  nav.TakeOff();
  while (ros::ok() && nav.Ok()) {
    int rc = venom::wait_key(0,1000,c);
    if (c == 'q' || rc < 0)
      break;
    //nav.SetPoint(pose);
    ros::Duration(0.5).sleep();
    ros::spinOnce();
  }
  nav.Land();
  return 0;
}
