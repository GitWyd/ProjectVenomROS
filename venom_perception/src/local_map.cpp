#include <ros/ros.h> // ros
#include <ros/console.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/io/pcd_io.h> // pcl
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl_conversions/pcl_conversions.h>
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
  float px,py,pz,qx,qy,qz,qw;
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
    px = stod(line.substr(8,line.size()));
    getline(file,line);
    py = stod(line.substr(8,line.size()));
    getline(file,line);
    pz = stod(line.substr(8,line.size()));

    getline(file,line);
    getline(file,line);
    qx = stod(line.substr(8,line.size()));
    getline(file,line);
    qy = stod(line.substr(8,line.size()));
    getline(file,line);
    qz = stod(line.substr(8,line.size()));
    getline(file,line);
    qw = stod(line.substr(8,line.size()));
    ROS_INFO_STREAM("position: " << px << ", " << py << ", " << pz);
    ROS_INFO_STREAM("quaternion: "<<qx<<", "<<qy<<", "<<qz<<", "<<qw);

    ros_pc2.header.frame_id = "/base_link";
    file.close();
  } else {
    ROS_ERROR_STREAM("Unable to open " << filename );
    return 1;
  }

  //ros::init(argc, argv, "local_map");
  //ros::NodeHandle nh;
  //ros::Publisher pc_pub = nh.advertise<sensor_msgs::PointCloud2>("/venom/local_map",10);
  //while (ros::ok()) {
  //  pc_pub.publish(ros_pc2);
    //ROS_INFO_STREAM(ros_pc2.header.frame_id);
  //  ros::spinOnce();
  //  ros::Duration(1.0).sleep();
  //}

  // Matrix transform
  pcl::PointCloud<pcl::PointXYZ>::Ptr source(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromPCLPointCloud2(pcl_pc2, *source);

  Eigen::Affine3f transform = Eigen::Affine3f::Identity();
  transform.translation() << 2.5, 0.0, 0.0;
  float theta = M_PI / 4.;
  transform.rotate (Eigen::AngleAxisf (theta, Eigen::Vector3f::UnitZ()));

  pcl::PointCloud<pcl::PointXYZ>::Ptr transformed(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::transformPointCloud (*source, *transformed, transform);

  transform = Eigen::Affine3f::Identity();
  theta = -M_PI / 6.;
  transform.translation() << 2.0, 0.0, 0.2;
  transform.rotate (Eigen::AngleAxisf (theta, Eigen::Vector3f::UnitZ()));
  pcl::PointCloud<pcl::PointXYZ>::Ptr transformed2(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::transformPointCloud (*source, *transformed2, transform);

  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> white (source, 255, 255, 255);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> red (transformed, 230, 20, 20); 
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> blue (transformed, 20, 20, 230); 
  pcl::visualization::PCLVisualizer viewer("Matrix transform");
  viewer.addPointCloud(source, white, "original");
  viewer.addPointCloud(transformed, red, "transformed");
  viewer.addPointCloud(transformed2, blue, "transformed2");
  while (!viewer.wasStopped()) {
    viewer.spinOnce();
  }


  return 0;
}
