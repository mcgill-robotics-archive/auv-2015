import roslib
import cv2
import rospy
import functools
import time
import voodoo_filter as voodoo
from cv_bridge import CvBridge, CvBridgeError
from std_msgs.msg import String
from sensor_msgs.msg import Image
import numpy as np
from matplotlib import pyplot as plt

##  import message file (definition located in .../computer_vision/msg)
from computer_vision.msg import ObjectImageLocation                 # check this out if you want to change the format of the message

##  import for external functions here
# follow format "from <file name without .py> import <filter_name>"
# ie
import template

class plumber :

    #################### Examples ####################
    # DO NOT USE THESE, THESE ARE ONLY SAMPLES FOR YOU TO DRAW INSPIRATION #

    # an example showing how to create and publish messages
    @staticmethod
    # start by defining a function with image and **box as parameters
    # if you want the publisher you just add publisher as a parameter
    # here I will show you how to access publisher through the ** box
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

    # example showing how to use external functions imported at the top
    @staticmethod                                               # nike, just do it
    def template(image, **box):                                 # nike this
        return template_filter.filter_image(image)              # this is how you call your function filter_image from imported template_filter file

    # example showing how to use external functions and how to use hexadatum 
    @staticmethod
    def template_with_output(image,**box):
        filtered_image = template_filter.filter_image(image)    # filter your image
        data = template_filter.generate_msg(image)              # this is a function that returns a hexadatum defined below
        # hexadatum generates the ros message for you
        # to access the ros message use data.msg
        box["publisher"].publish(data.msg)                      # publish the message
        #(note: "publisher" is a key to get the publisher, this is not naming anything ie you don't need to change this)
        return filtered_image                                   # return the image that you have modified

    @staticmethod
    def template_with_voodoo(image, **box):
        # use voodoo.magic to create an object from imported class template 
        vudu = voodoo.magic(template)                           # voodoo.magic takes a class and two optional parameters function_name and msg_generator_name (use these if you have non-default function names)
        box["publisher"].publish(vudu.msg_generator(image))     # nike this too
        return vudu.filter_function(image)                      # run the filter function from the class
    
    #################### EXAMPLE ENDS ####################

    # this is a runnable example to acquire addition parameters which can be passed from config files
    @staticmethod
    def resize(image, scale_x=0.333, scale_y=0.333, **box): #scale_x and scale_y can be taken from config files, please note that it is before **box     
        return cv2.resize(image, None, fx=scale_x, fy=scale_y)

    # here is a bad filter, it is bad and it can break the pipe. Try it, you'll see
    @staticmethod
    def bad_filter(image, **box):
        return "garbage"
    
    @staticmethod
    def gray(image, **box):
        return cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)  #change to gray scale

    @staticmethod
    def hsv(image, **box):
        return cv2.cvtColor(image, cv2.COLOR_BGR2HSV)   #change to HSV
    
    @staticmethod
    def normalize(image, **box):
        return cv2.normalize(image); #maximizes the contrast
    
    @staticmethod
    def gaussian(image, **box):
        return cv2.GaussianBlur(image, 15, 0); #Reduces noise

    @staticmethod
    def canny(image, threshold1=100, threshold2=200, **box):
        return cv2.Canny(image, threshold1, threshold2) #Opencv magic

    @staticmethod
    def colour(image, colour=0, channels=[0], bins=[180], value_range=[0, 180], **box):
        # generate histogram
        hist = cv2.calcHist( [plumber.hsv(image)], channels, None, bins, value_range )
        
        return image

   
# a class defining what the image parameter above are using
# this is used elsewhere so don't change this
# if you want to use this wrap around it svp
class filter_image(object):
    def __init__(self, cv_image):
        self.content = cv_image
        self.timestamp = time.time()

    def update_content(self, cv_image):
        """Setter for content"""
        self.content = cv_image

    def __cmp__(self, other):
        return self.timestamp - other.timestamp

class hexadatum(object):
    class Target(object):
        NOTHING = 0
        GATE = 1
        BUOY = 2
        LANE = 3
        pass
    def __init__(self, x,y,z,pitch,yaw,task,description=""):
        self.msg = ObjectImageLocation()
        self.msg.centre_x = x
        self.msg.centre_y = y
        self.msg.depth = z
        self.msg.pitch = pitch
        self.msg.yaw = yaw
        self.msg.task = task if task in range(4) else 0       # if invalid target is given defaults target to NOTHING
        self.msg.description = description

    def __repr__(self):
        return self.msg.__repr__()
    def __str__(self):
        return self.msg
