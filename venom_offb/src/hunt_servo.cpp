#include <ros/ros.h>
#include <std_msgs/Int32MultiArray.h>
#include <venom_offb/Navigator.h>
#include <venom_perception/Zed.h>
#include <eigen_conversions/eigen_msg.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>

int px1=0,px2=0,py1=0,py2=0;
//int cx = 360, cy = 540; // 720p
int cx = 128, cy = 128;
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
  while (ros::ok()) {
    geometry_msgs::Pose curr = zed.GetPose();
    //ROS_INFO_STREAM("ping " << curr);
    if (px1!=0 || px2!=0 || py1 != 0 || py2 != 0) {
      ROS_INFO_STREAM("current pose " << curr);
      Eigen::Affine3d t;
      tf::poseMsgToEigen (curr, t);
      int midx = (px1 + px2)/2, midy = (py1 + py2)/2;
      ROS_INFO_STREAM("target center (" << midx << ", " << midy << ")");
      if (midy - cy > 30 ) {
        ROS_INFO("Turn right");
        t.rotate (Eigen::AngleAxisd (-M_PI/4.0, Eigen::Vector3d::UnitZ()));
      } else if (midy - cy < -30 ) {
        ROS_INFO("Turn left");
        t.rotate (Eigen::AngleAxisd (M_PI/4.0, Eigen::Vector3d::UnitZ()));
      }
      t.translation() << curr.position.x, curr.position.y, curr.position.z;
      //if (midx - cx > 30 ) {
      //  ROS_INFO("Go up");
      //  t.translation() << curr.position.x, curr.position.y, curr.position.z+0.05;
      //} else if (midx - cx < -30 ) {
      //  ROS_INFO("Go down");
      //  t.translation() << curr.position.x, curr.position.y, curr.position.z-0.05;
      //}
      geometry_msgs::PoseStamped cmd;
      tf::poseEigenToMsg(t, cmd.pose);
      nav->SetPoint(cmd);

      px1 = py1 = px2 = py2 = 0; // clear buffered values
    }
    d.sleep();
    ros::spinOnce();
  }
  nav->TakeOff(1.0); // TODO: this is bad... try to redesign the class pattern
  nav->SetTolerence(0.1);
  ROS_INFO("Back to 1 meter high");
  while (ros::ok() && nav->GetStatus() != venom::NavigatorStatus::IDLE) {
    d.sleep();
    ros::spinOnce();
  }
  ROS_INFO("Landing...");
  nav->Land();
  return 0;
}
