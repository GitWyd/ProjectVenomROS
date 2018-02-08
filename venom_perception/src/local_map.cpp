#include <ros/ros.h> // ros
#include <ros/console.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/pcd_io.h> // pcl
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <string> // std
#include <fstream>

int main(int argc, char** argv) {
  if (argc < 3) {
    ROS_ERROR("Please specify files");
    return 1;
  }
  
  // Read .pcd file
  std::string filename(argv[1]);

  pcl::PCLPointCloud2 pcl_pc2;
  ROS_INFO_STREAM("Loading " << filename << "...");
  if (pcl::io::loadPCDFile (filename, pcl_pc2) == -1) {
    ROS_ERROR("Load failed");
    return 1;
  }
  ROS_DEBUG("Converting pcl to ros cloud format...");
  sensor_msgs::PointCloud2 ros_pc2;
  pcl_conversions::fromPCL(pcl_pc2, ros_pc2);
  ROS_INFO("Load succeed");

  // Read .txt file
  filename = argv[2];
  ROS_INFO_STREAM("Loading " << filename << "...");
  std::ifstream file (filename);
  if (file.is_open()) {
    std::string line;
    for (int i=0; i<7; i++) {
      getline(file,line);
      ROS_INFO_STREAM(i << ": " << line);
    }

    // Retrieve positional information
    getline(file,line);
    float px = stod(line.substr(8,line.size()));
    getline(file,line);
    float py = stod(line.substr(8,line.size()));
    getline(file,line);
    float pz = stod(line.substr(8,line.size()));

    getline(file,line);
    getline(file,line);
    float qx = stod(line.substr(8,line.size()));
    getline(file,line);
    float qy = stod(line.substr(8,line.size()));
    getline(file,line);
    float qz = stod(line.substr(8,line.size()));
    getline(file,line);
    float qw = stod(line.substr(8,line.size()));
    ROS_INFO_STREAM("position: " << px << ", " << py << ", " << pz);
    ROS_INFO_STREAM("quaternion: "<<qx<<", "<<qy<<", "<<qz<<", "<<qw);

    ros_pc2.header.frame_id = "/base_link";
    file.close();
  } else {
    ROS_ERROR_STREAM("Unable to open " << filename );
    return 1;
  }

  ros::init(argc, argv, "local_map");
  ros::NodeHandle nh;
  ros::Publisher pc_pub = nh.advertise<sensor_msgs::PointCloud2>("/venom/local_map",10);
  while (ros::ok()) {
    pc_pub.publish(ros_pc2);
    ROS_INFO_STREAM(ros_pc2.header.frame_id);
    ros::spinOnce();
    ros::Duration(1.0).sleep();
  }

  return 0;
}
