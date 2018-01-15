#include <ros/ros.h>
#include <std_msgs/Char.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <iostream>
#include <vector>
#include <signal.h>
#include <math.h>
#include <thread>

#ifndef VENOM_NAVIGATOR_H
#define VENOM_NAVIGATOR_H
namespace venom {

class VenomNavigator {
public:
  VenomNavigator() {

    setpoint_.pose.position.x = 0.0; 
    setpoint_.pose.position.y = 0.0;
    setpoint_.pose.position.z = 0.0;
    setpoint_.pose.orientation.x = 0.0;
    setpoint_.pose.orientation.y = 0.0;
    setpoint_.pose.orientation.z = 0.0;
    setpoint_.pose.orientation.w = 0.0;

    nav_active_ =false;

    // Initialize ROS objects
    ros::NodeHandle nh;
    arming_client_ = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    set_mode_client_ = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

    pose_sub_ = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 10, &VenomNavigator::PoseCallback, this);
    state_sub_ = nh.subscribe<mavros_msgs::State>("mavros/state", 10, &VenomNavigator::StateCallback, this);
    command_sub_ = nh.subscribe<std_msgs::Char>("/venom/high_level", 10, &VenomNavigator::CommandCallback, this);

    setpoint_pub_ = nh.advertise<geometry_msgs::PoseStamped>("mavros/setpoint_position/local", 10);

    while(ros::ok() && !state_.connected){
        ros::spinOnce();
        ros::Duration(0.5).sleep();
        ROS_INFO("Waiting for FCU connection");
    }
  }

  ~VenomNavigator() {
    pose_sub_.shutdown();
    state_sub_.shutdown();
    command_sub_.shutdown();
    Land();
  }

  bool Ok() {
      return state_.connected;
  }

  void TakeOff(double h = 1.0) {
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";
    set_mode_client_.call(offb_set_mode);

    ros::Time last_request = ros::Time::now();

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    setpoint_.pose.position.x = 0.0; 
    setpoint_.pose.position.y = 0.0;
    setpoint_.pose.position.z = h;
    setpoint_.pose.orientation.x = 0.0;
    setpoint_.pose.orientation.y = 0.0;
    setpoint_.pose.orientation.z = 0.0;
    setpoint_.pose.orientation.w = 0.0;

    ROS_INFO("Buffering setpoints (3 sec)...");
    ros::Duration d(0.1);
    for (int i = 0; i < 30; i++) {
      setpoint_pub_.publish(setpoint_);
      ros::spinOnce();
      d.sleep();
    }
    ROS_INFO("Buffer done");

    if (!state_.connected) {
      ROS_ERROR("Not connected");
      return;
    }
    if (set_mode_client_.call(offb_set_mode) && offb_set_mode.response.mode_sent )
      ROS_INFO("Offboard enabled");
    if (arming_client_.call(arm_cmd) && arm_cmd.response.success)
      ROS_INFO("Vehicle armed");
    if (InitNavProcess() )
      ROS_INFO("Init SUCCESS");
  }

  void Land() {
    if (!EndNavProcess()) {
      ROS_DEBUG("Thread not running");
      return;
    }
    setpoint_.pose.position.x = 0.0; // Racing condition!! Probably need to synchronize
    setpoint_.pose.position.y = 0.0;
    setpoint_.pose.position.z = 0.5;

    ROS_INFO("Landing...");
    ros::Duration d(0.1);
    while (Error(setpoint_) > tolerence) {
      setpoint_pub_.publish(setpoint_);
      ros::spinOnce();
      d.sleep();
    }

    if (!state_.connected) {
      ROS_ERROR("Not connected");
      return;
    }

    mavros_msgs::SetMode offb_set_mode;
    //offb_set_mode.request.custom_mode = "AUTO.LAND";
    offb_set_mode.request.custom_mode = "STABILIZED";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    if( set_mode_client_.call(offb_set_mode) && offb_set_mode.response.mode_sent )
      ROS_INFO("Switched to STABILIZED mode");
    else
      ROS_ERROR("Fail to switch mode");
  }

  void SetPoint(const geometry_msgs::PoseStamped& ps) {
    setpoint_ = ps;
  }

  double Error(geometry_msgs::PoseStamped pose) {
    double error = 0;
    error += fabs(pose_.pose.position.x - pose.pose.position.x);
    error += fabs(pose_.pose.position.y - pose.pose.position.y);
    error += fabs(pose_.pose.position.z - pose.pose.position.z);
    return error;
  }


private:
  mavros_msgs::State state_;
  geometry_msgs::PoseStamped pose_;
  geometry_msgs::PoseStamped setpoint_;
  char command_;

  ros::ServiceClient arming_client_;
  ros::ServiceClient set_mode_client_;

  ros::Subscriber pose_sub_;
  ros::Subscriber state_sub_;
  ros::Subscriber command_sub_;

  ros::Publisher setpoint_pub_;
  void StateCallback(const mavros_msgs::State::ConstPtr& msg){
    state_ = *msg;
  }

  void PoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg){
    pose_ = *msg;
  }

  void CommandCallback(const std_msgs::Char::ConstPtr& msg){
    command_ = msg->data;
  }

  // Separate thread to control setpoint_
  bool nav_active_ = false;
  std::thread navigate_;
  double tolerence = 0.15;

  bool InitNavProcess() {
    if (nav_active_) {
      ROS_DEBUG("Thread already running");
      return false;
    }
    nav_active_ = true;
    navigate_ = std::thread(&VenomNavigator::NavProcess, this);
    return true;
  }

  bool EndNavProcess() {
    if (!nav_active_) {
      ROS_DEBUG("Thread not running");
      return false;
    }
    nav_active_ = false;
    navigate_.join();
    return true;
  }

  void NavProcess() {
    ros::Duration d(0.1);
    geometry_msgs::PoseStamped nav_setpoint = setpoint_;
    while (ros::ok() && Ok() && nav_active_) {
      //ROS_DEBUG("NavProcess heartbeats");
      if (Error( nav_setpoint ) < tolerence)
	nav_setpoint = setpoint_;
      setpoint_pub_.publish(nav_setpoint);
      d.sleep();
    }
    if (nav_active_)
      ROS_WARN("NavProcess terminated with exception");
  }
};
} // namespace venom
#endif
