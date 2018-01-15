#include <iostream>
#include <signal.h>
#include "util.h"
#include "Navigator.h"

venom::VenomNavigator* nav;

void exit_handler(int s) {
  ROS_WARN("Force quitting...\n");
  nav->Land();
  delete nav;
  exit(1);
}

int main(int argc, char **argv) {

  ros::init(argc, argv, "Navigator", ros::init_options::NoSigintHandler);
  signal(SIGINT, exit_handler);
  char c = ' ';
  int rc;

  nav = new venom::VenomNavigator();
  nav->TakeOff(2.0);
  ros::Duration d(0.5);
  while (ros::ok() && nav->Ok()) {
    int rc = venom::wait_key(0,1000,c);
    if (c == 'q' || rc < 0)
      break;
    d.sleep();
    ros::spinOnce();
  }

  std::cout << "quit and land\n";
  nav->Land();
  delete nav;
  return 0;
}
