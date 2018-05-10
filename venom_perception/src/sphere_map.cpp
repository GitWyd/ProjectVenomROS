#include <ros/ros.h>
#include <fstream>
#include <vector>
#include <string>
#include <geometry_msgs/Pose.h>

#define DEBUG_MODE 1

int main(int argc, char **argv) {
  ros::init(argc, argv, "spherical_map");
  std::vector<std::string>posefiles({"pose_0.txt", "pose_1.txt", "pose_2.txt",
                                     "pose_3.txt", "pose_4.txt", "pose_5.txt"});
  std::vector<geometry_msgs::Pose> poses;
  for (std::string& posefile : posefiles) {
    #if DEBUG_MODE == 1
    ROS_INFO_STREAM("Reading " << posefile << "...");
    #endif

    std::ifstream txt(posefile);
    std::string line;
    geometry_msgs::Pose pose;
    if (txt.is_open()) {
      std::getline(txt, line);
      size_t pos0 = line.find(':');
      size_t pos1 = line.find(',',pos0+1);
      size_t pos2 = line.find(',',pos1+1);
      size_t pos3 = line.size();

      pose.position.x = stod(line.substr(pos0+1, pos1-pos0-1));
      pose.position.y = stod(line.substr(pos1+1, pos2-pos1-1));
      pose.position.z = stod(line.substr(pos2+1, pos3-pos2-1));

      std::getline(txt, line);
      pos0 = line.find(':');
      pos1 = line.find(',',pos0+1);
      pos2 = line.find(',',pos1+1);
      pos3 = line.find(',',pos2+1);
      size_t pos4 = line.size();
      pose.orientation.x = stod(line.substr(pos0+1, pos1-pos0-1));
      pose.orientation.y = stod(line.substr(pos1+1, pos2-pos1-1));
      pose.orientation.z = stod(line.substr(pos2+1, pos3-pos2-1));
      pose.orientation.w = stod(line.substr(pos3+1, pos4-pos3-1));
      #if DEBUG_MODE == 1
      ROS_INFO_STREAM(pose);
      #endif
    } else {
      ROS_ERROR_STREAM("fail to read " << posefile);
      return 1;
    }
  }
  return 0;
}
