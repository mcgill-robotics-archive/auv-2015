#!/usr/bin/env python
import roslib
import sys
import rospy
import cv2
import pipe_manager
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
class image_in:
  def __init__(self, subscribe_to, pipe_curry):
    self.image_pub = rospy.Publisher("cvimage_cam_a_raw",Image, queue_size=10)
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber(subscribe_to,Image,self.callback)
    self.curry = pipe_curry
    
  def callback(self,data):
    print "Calling back"
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError, e:
      print e
    sauce_image = pipe_manager.image(cv_image)
    print "Giving pipe image"
    full = self.curry(sauce_image)()
    ## display to gui
##    cv2.imshow("Image window", full.stuffing)
##    cv2.waitKey(5)

