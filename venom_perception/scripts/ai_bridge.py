#!/usr/bin/env python
import roslib
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import sys, os
sys.path.append(os.path.join(os.getcwd(),'python/'))
import darknet as dn
import time

def array_to_image(arr):
    arr = arr.transpose(2,0,1)
    c = arr.shape[0]
    h = arr.shape[1]
    w = arr.shape[2]
    arr = (arr/255.0).flatten()
    data = dn.c_array(dn.c_float, arr)
    im = dn.IMAGE(w,h,c,data)
    return im

def detect2(net, meta, image, thresh=.5, hier_thresh=.5, nms=.45):
    boxes = dn.make_boxes(net)
    probs = dn.make_probs(net)
    num =   dn.num_boxes(net)
    dn.network_detect(net, image, thresh, hier_thresh, nms, boxes, probs)
    res = []
    for j in range(num):
        for i in range(meta.classes):
            if probs[j][i] > 0:
                res.append((meta.names[i], probs[j][i], (boxes[j].x, boxes[j].y, boxes[j].w, boxes[j].h)))
    res = sorted(res, key=lambda x: -x[1])
    dn.free_ptrs(dn.cast(probs, dn.POINTER(dn.c_void_p)), num)
    return res


class ImageReader:
  def __init__(self):
    self.__bridge = CvBridge()
    self.__rgb_sub = rospy.Subscriber("/zed/rgb/image_rect_color",Image,self.__rgb_cb)
    self.__depth_sub = rospy.Subscriber("/zed/depth/depth_registered",Image,self.__depth_cb)
    self.rgb_image = None
    self.depth_image = None
    self.net = None #dn.load_net(b"drone_data/tiny-yolo-drone.cfg", b"drone_data/tiny-yolo-drone.backup", 0)
    self.meta = None #dn.load_meta(b"drone_data/drone1.data")
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

def main():
  ir = ImageReader()
  #print "Press [Anykey] to grab image"
  #print "Press [q] to quit"

  net = dn.load_net(b"drone_data/tiny-yolo-drone.cfg", b"drone_data/tiny-yolo-drone.backup", 0)
  meta = dn.load_meta(b"drone_data/drone1.data")
  total_fps = 0
  while(True):
    image = ir.get_rgb(256,256)
    start = time.time()
    im = array_to_image(image)
    r = detect2(net, meta, im, thresh=0.2, hier_thresh=0.5, nms=0.45)
    avg = [0]*4
    weight = 0.0
    for region in r:
        if float('inf') not in region[2]:
            for e in range(len(region[2])):
                avg[e] += region[2][e]*region[1]
            weight += region[1]
    if len(r) > 0:
        for e in range(len(avg)): avg[e] /= weight
    print(avg)
    if weight > 0:
        cv2.rectangle(image, (int(avg[0]), int(avg[1])), (int(avg[0]+avg[2]), int(avg[1]+avg[3])), (0,0,255))
    fps = 1./(time.time()-start)
    print(fps)

    cv2.imshow('darknet output',image)
    c = cv2.waitKey(3)
    if c == ord('q'):
      break

  cv2.destroyAllWindows()

    
    

if __name__ == '__main__':
  main()