import roslib
import cv2
import rospy
import functools
import time
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import String
from sensor_msgs.msg import Image
from computer_vision.msg import ObjectImageLocation
import numpy as np
from matplotlib import pyplot as plt


class plumber :

    @staticmethod
    # start by defining a function with image and **box as parameters
    # if you want the publisher you just add publisher as a parameter
    # here I will show you how to access publisher throught the **box
    # ie the following is same as def example_filter(image,publisher)
    def example_filter(image, **box):
        # do some operation on the image
        ### DO SOMETHING HERE ###
        # setup message
        location = ObjectImageLocation()
        # here you set the location of the object
        location.centre_x = 0
        location.centre_y = 0
        # publish the message
        box["publisher"].publish(location)
        # return the modified image
        return image
    
    @staticmethod
    def gray(image, **box):
        return cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    @staticmethod
    def hsv(image, **box):
        return cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    
    @staticmethod
    def resize(image, scale_x=0.333, scale_y=0.333, **box):
        return cv2.resize(image, None, fx=scale_x, fy=scale_y)
    
    @staticmethod
    def normalize(image, **box):
        return cv2.normalize(image); #maximizes the contrast
    
    @staticmethod
    def gaussian(image, **box):
        return cv2.GaussianBlur(image, 15, 0); #Reduces noise

    @staticmethod
    def canny(image, threshold1=100, threshold2=200, **box):
        return cv2.Canny(image, threshold1, threshold2)

    @staticmethod
    def colour(image, colour=0, channels=[0], bins=[180], value_range=[0, 180], **box):
        # generate histogram
        hist = cv2.calcHist( [plumber.hsv(image)], channels, None, bins, value_range )
        
        return image

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
    def message(image):
        resized = cv2.resize(image, (0,0), fx=0.5, fy=0.5)
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
