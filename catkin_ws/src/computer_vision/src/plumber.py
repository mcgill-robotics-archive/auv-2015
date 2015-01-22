import roslib
import cv2
import rospy
import functools
import time
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import String
from sensor_msgs.msg import Image


class plumber :
    @staticmethod
    def gray(image):
        return cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    @staticmethod
    def line_detection(image) :
        print "received image ", image
        return cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    
    @staticmethod
    def resize(image, scale_x=0.333, scale_y=0.333):
        return cv2.resize(image, (0,0), fx=scale_x, fy=scale_y)

    @staticmethod
    def bad_filter(image):
        return "garbage"

    @staticmethod
    def curry(filters):
        return reduce(lambda f1, f2: lambda image: f1(f2(image)), filters)
    
    @staticmethod
    def f3():
        return lambda image:image
    @staticmethod
    def f4():
        return lambda image:image
    @staticmethod
    def end():
        return rospy.Publisher("line",Image, queue_size=10)
    @staticmethod
    def message(bridge, image):
        print "message"
        resized = cv2.resize(image.content, (0,0), fx=0.5, fy=0.5)
        cv2.imshow("Image window", resized)
        return bridge.cv2_to_imgmsg(image.content , "bgr8")

class filter_image(object):
    def __init__(self, cv_image):
        self.content = cv_image
        self.timestamp = time.time()

    def update_content(self, cv_image):
        """Setter for content"""
        self.content = cv_image

    def __cmp__(self, other):
        return self.timestamp - other.timestamp
