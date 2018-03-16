#!/usr/bin/env python

# Simple testing program passing bounding boxes from neural network
import rospy
from std_msgs.msg import Int32MultiArray

def bb_callback(msg):
  rospy.loginfo('Receive bounding box (%i, %i) to (%i, %i)', msg.data[0], \
                msg.data[1], msg.data[2], msg.data[3])

rospy.init_node('ai_publisher',anonymous=True)
pub = rospy.Publisher('/venom/bounding_box', Int32MultiArray, queue_size=10)
sub = rospy.Subscriber('/venom/bounding_box', Int32MultiArray, bb_callback)
while not rospy.is_shutdown():
  out = Int32MultiArray()
  out.data = [0, 255, 3, 30]
  pub.publish(out)
  rospy.loginfo('heartbeat')
  rospy.sleep(0.5)
