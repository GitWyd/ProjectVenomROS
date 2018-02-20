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
#include <vector>
#include <glob.h>
#include <Eigen/Dense> // eigen
#include <Eigen/Geometry> 


// https://stackoverflow.com/questions/8401777/simple-glob-in-c-on-unix-system 
inline std::vector<std::string> glob(const std::string& pat){
  glob_t glob_result;
  glob(pat.c_str(),GLOB_TILDE,NULL,&glob_result);
  std::vector<std::string> ret;
  for(unsigned int i=0;i<glob_result.gl_pathc;++i){
      ret.push_back(std::string(glob_result.gl_pathv[i]));
  }
  globfree(&glob_result);
  return ret;
}


int read_record(std::string cloud_src,
                pcl::PointCloud<pcl::PointXYZRGB>& cloud,
		std::string pose_src,
                Eigen::Vector3f& translation,
		Eigen::Affine3f& transform) {
  ROS_INFO_STREAM("Loading " << cloud_src << "...");
  pcl::PCLPointCloud2 pcl_pc2;
  if (pcl::io::loadPCDFile (cloud_src, pcl_pc2) == -1) {
    ROS_ERROR("Load failed");
    return 1;
  }
  pcl::fromPCLPointCloud2(pcl_pc2, cloud);

  ROS_INFO_STREAM("Loading " << pose_src << "...");
  std::ifstream file (pose_src);
  if (file.is_open()) {
    std::string line;
    for (int i=0; i<7; i++) {
      getline(file,line);
    }

    // Retrieve positional information
    getline(file,line);
    translation[0] = stod(line.substr(8,line.size()));
    getline(file,line);
    translation[1] = stod(line.substr(8,line.size()));
    getline(file,line);
    translation[2] = stod(line.substr(8,line.size()));

    // Rotate transform by quaternion
    transform = Eigen::Affine3f::Identity();
    getline(file,line);getline(file,line);
    float qx = stod(line.substr(8,line.size()));
    getline(file,line);
    float qy = stod(line.substr(8,line.size()));
    getline(file,line);
    float qz = stod(line.substr(8,line.size()));
    getline(file,line);
    float qw = stod(line.substr(8,line.size()));
    Eigen::Quaternion<float> q(qw,qx,qy,qz);
    
    transform.rotate(q);
    transform = transform.inverse();

    file.close();
  } else {
    ROS_ERROR_STREAM("Unable to open " << pose_src << ".txt" );
    return 1;
  }
  return 0;
}

int main(int argc, char** argv) {

  std::vector<std::string> clouds({"samples/cloud_15.pcd",
                                   "samples/cloud_20.pcd",
                                   "samples/cloud_25.pcd",
                                   "samples/cloud_30.pcd"});
  std::vector<std::string> poses({"samples/pose_15.txt",
                                  "samples/pose_20.txt",
                                  "samples/pose_25.txt",
                                  "samples/pose_30.txt"});
  pcl::visualization::PCLVisualizer viewer("Matrix transform");

  for (int i = 0; i < 4; i++) {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    Eigen::Vector3f translation;
    Eigen::Affine3f transform;
    read_record(clouds[i],*cloud, poses[i], translation, transform);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed(new pcl::PointCloud<pcl::PointXYZRGB>);
    transform.translation() << translation;
    pcl::transformPointCloud (*cloud, *transformed, transform);

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> white (cloud, 255, 255, 255);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> red (transformed, 230, 20, 20); 
    //viewer.addPointCloud(cloud, white, "original"+std::to_string(i));
    viewer.addPointCloud(transformed, "transformed"+std::to_string(i));
  }
  //viewer.addPointCloud(transformed2, blue, "transformed2");
  while (!viewer.wasStopped()) {
    viewer.spinOnce();
  }


  return 0;
}
