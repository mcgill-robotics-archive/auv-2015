#!/usr/bin/env python
import roslib
import sys
import rospy
import cv2
import urllib 
import numpy as np
import argparse
import Queue
import time
import thread
from threading import Thread
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

if __name__ == '__main__':
    rospy.init_node("phone_cam", anonymous=True)
    bridge = CvBridge()
    parser = argparse.ArgumentParser(description="Create an image processing pipeline in ROS system")
    parser.add_argument("ip", help="ip address" )
    parser.add_argument("-t","--topic", help="Set the topic name of the published messages (default : phone_cam)")
    parser.add_argument("-s", "--show", help="Show the camera", action="store_true")
    arguments = vars(parser.parse_args())
    ip = arguments["ip"]
    topic_name = arguments["topic"] if arguments["topic"] else "phone_cam"
    image_publisher = rospy.Publisher(topic_name,Image)
    location = "http://" + ip + "//video?dummy=param.mjpg"
    stream=urllib.urlopen(location)
    queue = Queue.Queue()
    print "Press q to quit"
    def read_cam():
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
                queue.put_nowait(cv_image)
    def show():
        while True:
            cv_image = queue.get(block=True)
            cv2.imshow('image port',cv_image)
            if cv2.waitKey(5) == ord('q'):
                # signal main thread to close down
                thread.interrupt_main()
                exit(0)
    cam_thread = Thread(target=read_cam)
    cam_thread.daemon = True
    cam_thread.start()
    if arguments["show"] :
        try:
            gui_thread = Thread(target=show)
            gui_thread.daemon = True
            gui_thread.start()
        except KeyboardInterrupt:
            rospy.exit()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down"
    cv2.destroyAllWindows()
