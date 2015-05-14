#! /usr/bin/env python2.7
'''
Author: Max Krogius

CV util library to handle setting up a ros node and applying common filters. 
Add new filters in the filters folder. See the example for usage

Subscriptions:
    /image [sensor_msgs/Image] : For the input image
'''

import rospy
import filters
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class CvBackbone(object):

    def __init__(self, name):
        rospy.init_node(name)
        self.name = name
        self.sub = rospy.Subscriber('image', Image, self.onImageReceived)
        self.bridge = CvBridge()
        self.imageCallbacks = []
        self.filters = filters
        self.publishers = {}

    def onImageReceived(self, img):
        img = self.bridge.imgmsg_to_cv2(img)
        for callback in self.imageCallbacks:
            callback(img)
    
    def addImageCallback(self, callback):
        self.imageCallbacks.append(callback)

    def publishImage(self, img, name=None):
        if not name:
            name = self.name + '/image'
        pub = self.publishers.get(name, None)
        if not pub:
            pub = rospy.Publisher(name, Image, queue_size=10)
            self.publishers[name] = pub
        pub.publish(self.cv2_to_imgmsg(img))

    def cv2_to_imgmsg(self, img):
        # cv_bridge does not by default correctly transform mono images so
        # here we make sure to publish them in a ros-compatible format
        if len(img.shape) == 3:
            return self.bridge.cv2_to_imgmsg(img, 'bgr8')
        elif len(img.shape) == 2:
            return self.bridge.cv2_to_imgmsg(img, 'mono8')
        else:
            print('Image has wrong number of channels')

    def start(self):
        rospy.spin()
