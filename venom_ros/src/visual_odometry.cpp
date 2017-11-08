#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <tf/transform_datatypes.h>
#include <list>
#include <vector>
#include <iostream>
#include <string>
#include <fstream>

double x,y,z,roll,pitch,yaw;

void odom_callback(const nav_msgs::Odometry& msg){
	// Update XYZ
	x = msg.pose.pose.position.x;
	y = msg.pose.pose.position.y;
	z = msg.pose.pose.position.z;

	// Update RPY
	geometry_msgs::Quaternion o = msg.pose.pose.orientation;
	tf::Quaternion q (o.x, o.y, o.z, o.w );
	tf::Matrix3x3 m(q);
	m.getRPY(roll, pitch, yaw);

	return;
}

int main(int argc, char** argv){
	ros::init(argc,argv,"odometry_tracker");
	ros::NodeHandle n;
	ros::Subscriber odom = n.subscribe("/zed/odom",5,odom_callback);

	ros::Rate loop_rate(1);

	std::list<double> route; // planned route
	std::string s;
	double d;
	while(true){
		std::cin>>d;
		if (d==-99)
			break;
		route.push_back(d);
		ros::spinOnce();
	};
	int num_iter;
	std::cout<<"Enter number of iterations: ";
	std::cin>>num_iter;
	std::list<std::vector<double> > data;
	for(int i=0;i<num_iter;++i){
		std::cout<<"\nIteration "<<i+1<<std::endl;
		std::cin.ignore();
		for(auto pt=route.begin();pt!=route.end();++pt){
			ros::spinOnce();
			std::cout<<"Please move camera to "<<*pt;
			std::cin.ignore();
			std::vector<double> measure={*pt,x,y,z,roll,pitch,yaw};
			data.push_back(measure);
			ROS_INFO("x, y, z = [%f,%f,%f] roll, pitch, yaw = [%f,%f,%f]", x,y,z, roll, pitch, yaw);
		}
	}

	std::ofstream csvfile ("test.csv");
	if (csvfile.is_open()){
		csvfile << "command,x,y,z,roll,pitch,yaw\n";
		for (auto row=data.begin(); row!=data.end();++row){
			for (auto entry= (*row).begin();entry!=(*row).end();++entry)
				csvfile << *entry<< ',';
			csvfile << '\n';
		}
		csvfile<<'\n';
	}
	csvfile.close();
	return 0;
}
