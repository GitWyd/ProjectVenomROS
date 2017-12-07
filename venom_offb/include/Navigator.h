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

#ifndef VENOM_NAVIGATOR_H
#define VENOM_NAVIGATOR_H
namespace venom {

class VenomNavigator {
public:
  VenomNavigator() {
    ros::NodeHandle nh;

    // Initialize ROS objects
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
    //setpoint_pub_.unregister();
  }

  bool Ok() {
    return state_.connected;
  }

  void TakeOff() {
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";
    set_mode_client_.call(offb_set_mode);

    ros::Time last_request = ros::Time::now();

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0.0; 
    pose.pose.position.y = 0.0;
    pose.pose.position.z = 3.0;

    for (int i = 0; i < 30; i++) {
      setpoint_pub_.publish(pose);
      ros::spinOnce();
      ros::Duration(0.1);
      ROS_INFO("Buffering");
    }

    if (!state_.connected) {
      ROS_INFO("Not connected");
      return;
    }
    if( set_mode_client_.call(offb_set_mode) && offb_set_mode.response.mode_sent )
      ROS_INFO("Offboard enabled");
    if( arming_client_.call(arm_cmd) && arm_cmd.response.success)
      ROS_INFO("Vehicle armed");
  }

  void Land() {
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = 0.0; 
    pose.pose.position.y = 0.0;
    pose.pose.position.z = 0.0;

    while (Error(pose) > 0.05) {
      setpoint_pub_.publish(pose);
      ros::spinOnce();
      ros::Duration(0.1).sleep();
      ROS_INFO("Landing");
    }

    if (!state_.connected) {
      ROS_INFO("Not connected");
      return;
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "Manual";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    if( arming_client_.call(arm_cmd) && arm_cmd.response.success)
      ROS_INFO("Vehicle disarmed");
    if( set_mode_client_.call(offb_set_mode) && offb_set_mode.response.mode_sent )
      ROS_INFO("Back to manual mode");
  }

  void SetPoint(const geometry_msgs::PoseStamped& ps) {
    setpoint_pub_.publish(ps);
  }

  double Error(geometry_msgs::PoseStamped pose){
    double error = 0;
    error += abs(pose_.pose.position.x - pose.pose.position.x);
    error += abs(pose_.pose.position.y - pose.pose.position.y);
    error += abs(pose_.pose.position.z - pose.pose.position.z);
    return error;
  }


private:
  mavros_msgs::State state_;
  geometry_msgs::PoseStamped pose_;
  char command_;

  ros::ServiceClient arming_client_;
  ros::ServiceClient set_mode_client_;

  ros::Subscriber pose_sub_;
  ros::Subscriber state_sub_;
  ros::Subscriber command_sub_;

  ros::Publisher setpoint_pub_;
  void StateCallback(const mavros_msgs::State::ConstPtr& msg){
    state_ = *msg;
    //std::cout << "Echo from state callback\n";
  }

  void PoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg){
    pose_ = *msg;
    //std::cout << "Echo from pose callback\n";
  }

  void CommandCallback(const std_msgs::Char::ConstPtr& msg){
    command_ = msg->data;
  }
};
} // namespace venom
#endif
