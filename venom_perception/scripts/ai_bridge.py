#!/usr/bin/env python
import roslib
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


class ImageReader:
  def __init__(self):
    self.__bridge = CvBridge()
    self.__rgb_sub = rospy.Subscriber("/zed/rgb/image_rect_color",Image,self.__rgb_cb)
    self.__depth_sub = rospy.Subscriber("/zed/depth/depth_registered",Image,self.__depth_cb)
    self.rgb_image = None
    self.depth_image = None
    rospy.init_node('image_reader', anonymous=True)

  def __rgb_cb(self,data):
    try:
      self.rgb_image = self.__bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

  def __depth_cb(self,data):
    try:
      self.depth_image = self.__bridge.imgmsg_to_cv2(data, desired_encoding="32FC1")
    except CvBridgeError as e:
      print(e)

  def get_rgb(self, width = 256, height = 144):
    rgb = cv2.resize(self.rgb_image,dsize =(width, height) )
    return rgb

  def get_depth(self, width = 256, height = 144):
    depth = cv2.resize(self.depth_image,dsize =(width, height) )
    return depth

def demo_program():
  ir = ImageReader()
  print "Press [Anykey] to grab image"
  print "Press [q] to quit"
  while True:
    rgb = ir.get_rgb()
    cv2.imshow('RGB', rgb);
    depth = ir.get_depth()
    cv2.imshow('Depth', depth);
    c = cv2.waitKey(0);
    if c == ord('q'):
        break
  cv2.destroyAllWindows();


if __name__ == '__main__':
  demo_program()
