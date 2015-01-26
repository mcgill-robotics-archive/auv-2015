#!/usr/bin/env python
import roslib
import sys
import rospy
import cv2
import urllib 
import numpy as np
import argparse
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

if __name__ == '__main__':
    rospy.init_node("phone_cam", anonymous=True)
    image_publisher = rospy.Publisher("phone_cam_2",Image)
    bridge = CvBridge()
    parser = argparse.ArgumentParser(description="Create an image processing pipeline in ROS system")
    parser.add_argument("ip", help="ip address" )
    arguments = vars(parser.parse_args())
    ip = arguments["ip"]
    location = "http://" + ip + "//video?dummy=param.mjpg"
    stream=urllib.urlopen(location)
    bytes=''
    while True:
        bytes+=stream.read(16384)
        a = bytes.find('\xff\xd8')
        b = bytes.find('\xff\xd9')
        if a!=-1 and b!=-1:
            jpg = bytes[a:b+2]
            bytes= bytes[b+2:]
            cv_image = cv2.imdecode(np.fromstring(jpg, dtype=np.uint8),cv2.CV_LOAD_IMAGE_COLOR)
            image_publisher.publish(bridge.cv2_to_imgmsg(cv_image, "bgr8"))
            cv2.imshow('image port',cv_image)
            if cv2.waitKey(5) == ord('q'):
                exit(0) 
