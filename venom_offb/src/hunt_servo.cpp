#include <ros/ros.h>
#include <std_msgs/Int32MultiArray.h>
#include <venom_offb/Navigator.h>
#include <venom_perception/Zed.h>
#include <eigen_conversions/eigen_msg.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>

int px1=0,px2=0,py1=0,py2=0;
//int cx = 360, cy = 540; // 720p
//int cx = 128, cy = 128; // cv2.resize to 256x256
int cx = 320, cy = 240; // VGA
int tolx = cx/12, toly = cy/12;
bool trigger = false;
static void bb_callback(std_msgs::Int32MultiArray::ConstPtr msg) {
  px1 = msg->data[0];
  py1 = msg->data[1];
  px2 = msg->data[2];
  py1 = msg->data[3];
  trigger = true;
}

venom::Navigator* nav;                                                          
                                                                                
void exit_handler(int s) {                                                      
  ROS_WARN("Force quitting...\n");                                              
  nav->Land();                                                                  
  delete nav;                                                                   
  exit(1);                                                                      
}

int main (int argc, char** argv) {
  ros::init(argc, argv, "visual_servo", ros::init_options::NoSigintHandler);
  signal(SIGINT, exit_handler);
  ros::NodeHandle nh;
  ros::Subscriber bb_sub = nh.subscribe<std_msgs::Int32MultiArray>("/venom/bounding_box", 1, bb_callback);

  venom::Zed zed;
  zed.Enable(venom::PerceptionType::ODOM);

  nav = new venom::Navigator();
  nav->TakeOff(2.0);

  ros::Duration d(0.5);
  ROS_INFO("Searching target...");
  geometry_msgs::PoseStamped cmd;
  cmd.pose.position.x = 0.0;
  cmd.pose.position.y = 0.0;
  cmd.pose.position.z = 2.0;
  cmd.pose.orientation.x = 0.0;
  cmd.pose.orientation.y = 0.0;
  cmd.pose.orientation.z = 0.0;
  cmd.pose.orientation.w = 1.0;
  Eigen::Affine3d t;
  while (ros::ok() && px1==0 && px2==0 && py1==0 && py2==0) {
    tf::poseMsgToEigen (cmd.pose, t);
    t.rotate (Eigen::AngleAxisd (M_PI/10.0, Eigen::Vector3d::UnitZ()));
    tf::poseEigenToMsg(t, cmd.pose);
    nav->SetPoint(cmd);
    ros::spinOnce();
    d.sleep();
  }
  ROS_INFO("Begin z-axis following");
  while (ros::ok()) {
    if (px1!=0 || px2!=0 || py1 != 0 || py2 != 0) {
      // Compute the center of bounding box
      int midx = (px1 + px2)/2, midy = (py1 + py2)/2;
      ROS_INFO_STREAM("target center (" << midx << ", " << midy << ")");

      tf::poseMsgToEigen (cmd.pose, t);
      //TODO: uncomment if moving forward
      //t.translation() << curr.position.x+0.2, curr.position.y, curr.position.z;
      if (midx - cx > tolx ) {
        ROS_INFO("Go up");
        t.translation() << cmd.pose.position.x, cmd.pose.position.y, cmd.pose.position.z+0.05;
      } else if (midx - cx < -tolx ) {
        ROS_INFO("Go down");
        t.translation() << cmd.pose.position.x, cmd.pose.position.y, cmd.pose.position.z-0.05;
      }

      if (midy - cy > toly ) {
        ROS_INFO("Turn right");
        t.rotate (Eigen::AngleAxisd (-M_PI/10.0, Eigen::Vector3d::UnitZ()));
      } else if (midy - cy < -toly ) {
        ROS_INFO("Turn left");
        t.rotate (Eigen::AngleAxisd (M_PI/10.0, Eigen::Vector3d::UnitZ()));
      }
      // Publish command
      tf::poseEigenToMsg(t, cmd.pose);
      nav->SetPoint(cmd);

      px1 = py1 = px2 = py2 = 0; // clear buffered values
    }
    d.sleep();
    ros::spinOnce();
  }
  nav->Land(0.8);
  return 0;
}
