/**
 * @file offb_node.cpp
 * @brief Offboard control example node, written with MAVROS version 0.19.x, PX4 Pro Flight
 * Stack and tested in Gazebo SITL
 */

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <iostream>
#include <signal.h>
#include <math.h>

mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

geometry_msgs::PoseStamped current_pose;
void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    current_pose = *msg;
}

static std::list<geometry_msgs::PoseStamped>
CircleTrajectory(int resolution, double radius, double height){
    std::list<geometry_msgs::PoseStamped> traj;
    double tick = 2.0 * M_PI / resolution;
    double theta = 0.0;
    for (double i = 0; i < resolution; ++i){
        theta += tick;
        geometry_msgs::PoseStamped pose;
        pose.pose.position.x = radius * cos(theta);
        pose.pose.position.y = radius * sin(theta);
        pose.pose.position.z = height;
        traj.push_back( pose );
    }
    return traj;
}

static double EstimatedError(geometry_msgs::PoseStamped pose){
    double error = 0;
    std::cout << "Current pose: " << current_pose.pose.position.x
	    << ", " << current_pose.pose.position.y 
	    << ", " << current_pose.pose.position.z << std::endl;
    std::cout << "Command pose: " << pose.pose.position.x
	    << ", " << pose.pose.position.y 
	    << ", " << pose.pose.position.z << std::endl;
    error += abs(current_pose.pose.position.x - pose.pose.position.x);
    error += abs(current_pose.pose.position.y - pose.pose.position.y);
    error += abs(current_pose.pose.position.z - pose.pose.position.z);
    return error;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "offb_node");
    ros::NodeHandle nh;

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>
            ("mavros/state", 10, state_cb);
    ros::Subscriber pose_sub = nh.subscribe<geometry_msgs::PoseStamped>
            ("mavros/local_position/pose", 10, pose_cb);
    ros::Publisher local_pos_pub = nh.advertise<geometry_msgs::PoseStamped>
            ("mavros/setpoint_position/local", 10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>
            ("mavros/cmd/arming");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>
            ("mavros/set_mode");

    //the setpoint publishing rate MUST be faster than 2Hz
    double hz = 5.0;
    ros::Rate rate(hz);

    // wait for FCU connection
    while(ros::ok() && current_state.connected){
        ros::spinOnce();
        rate.sleep();
    }

    geometry_msgs::PoseStamped pose;
    pose = current_pose;
    pose.pose.position.x = 0.0;
    pose.pose.position.y = 0.0;
    pose.pose.position.z = 3.2;

    int res = 40;
    double r = 3.0, h = 3.0, tol = 0.001;
    std::list<geometry_msgs::PoseStamped> path = CircleTrajectory(res, r, h);

    //send a few setpoints for 3 seconds
    for(int i = 3.0*hz; ros::ok() && i > 0; --i){
        local_pos_pub.publish(pose);
        ros::spinOnce();
        rate.sleep();
    }


    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";
    set_mode_client.call(offb_set_mode);

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    ros::Time last_request = ros::Time::now();
    pose = path.front();

    while(ros::ok()){
        if( current_state.mode != "OFFBOARD" &&
            (ros::Time::now() - last_request > ros::Duration(5.0))){
            if( set_mode_client.call(offb_set_mode) &&
                offb_set_mode.response.mode_sent){
                ROS_INFO("Offboard enabled");
            }
            last_request = ros::Time::now();
        } else {
            if( !current_state.armed &&
                (ros::Time::now() - last_request > ros::Duration(5.0))){
                if( arming_client.call(arm_cmd) &&
                    arm_cmd.response.success){
                    ROS_INFO("Vehicle armed");
                }
                last_request = ros::Time::now();
            }
        }

        double e = EstimatedError( pose );
        if ( e < tol ){
            std::cout << "Switch to next set point\n";
	    path.push_back( pose );
            pose = path.front();
	    path.pop_front();
	}
	
        local_pos_pub.publish(pose);

        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
