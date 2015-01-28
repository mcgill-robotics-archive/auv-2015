#!/usr/bin/env python
import argparse
import rospy
import Queue
import cv2
import thread
import itertools
import numpy as np
import matplotlib.pyplot as plt
from threading import Thread
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from operator import itemgetter

class histogram :
    @staticmethod
    def hsv_hist_item(image):
        # get hsv space for image
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        return cv2.calcHist([hsv],[0],None,[256],[0,256])
    
    @staticmethod
    def high_hue_regions(image, min_threshold):
        blur = cv2.GaussianBlur(image, (15,15), 0)
        hist_item = histogram.hsv_hist_item(blur)
        # look for largest area under the curve for k values
        flattened_hist = list(itertools.chain.from_iterable(hist_item))
        flattened_hist = flattened_hist[:-76]
        # find area under "curves" of size of k in hue space
        k = 9
        # convolve over the hsv space with selector like kernel
        areas = np.convolve(flattened_hist, [1.]*k, "same")
        convolved_values = [(index + int(k/2), region) for index ,region in enumerate(areas) if region > min_threshold]
        # collect contiguous regions
        prev_index = 0
        current_peak = []
        peaks = []
        for (index, value) in convolved_values :
            if index is prev_index +1 :
                current_peak.append((index,value))
            else :
                peaks.append(current_peak)
                current_peak = [(index,value)]
            prev_index = index
        peaks.append(current_peak)
        # reduce each sublist to their max value
        max_peaks = [max(region, key=itemgetter(1)) for region in peaks if region]
        # opencv uses 180 degrees instead of conventional 360 so we multiply by 2
        max_peaks = [(2*index, value) for (index,value) in max_peaks]
        return max_peaks

    @staticmethod
    def display_histogram(image):
        hist_item = histogram.hsv_hist_item(image)
        # for histogram display
        bins = np.arange(256).reshape(256,1)
        cv2.normalize(hist_item,hist_item,0,256,cv2.NORM_MINMAX)
        histodata=np.int32(np.around(hist_item))
        pts = np.column_stack((bins,histodata))
        h = np.zeros((256,256,3))
        cv2.polylines(h,[pts],False, (255,0,0))
        h=np.flipud(h)
        cv2.imshow('Histogram',h)
        cv2.imshow("Original" , image)
        return
    
    @staticmethod
    def closest(image, hue, min_threshold=3000):
        #print histogram.high_hue_regions(image, min_threshold)
        return min([((hue - index)%360,(index,value)) for (index,value) in histogram.high_hue_regions(image, min_threshold)], key=itemgetter(1))[1]
    
    
if __name__=="__main__":
    print "started"
    parser = argparse.ArgumentParser(description="Colour finder")
    parser.add_argument("hue", type=int, help="Locate the peak closest to this hue value")
    parser.add_argument("-s", "--source", help="Image feed source (default: /camera/image_rect_color)")
    parser.add_argument("-f", "--file", help="Image source is a file", action="store_true")
    parser.add_argument("--threshold", type=int, help="Specify a minimum threshold for a hue peak")
    parser.add_argument("--show", help="Shows the source image and the histogram", action="store_true")
    arguments = vars(parser.parse_args())
    show = arguments["show"]
    # setup arguments
    args = {"hue":arguments["hue"]}
    if arguments["threshold"]:
        args["min_threshold"] = arguments["threshold"]
    if not arguments["file"]:
        camera = arguments["source"] if arguments["source"] else "/camera/image_rect_color"
        bridge = CvBridge()
        def callback(data):
            try:
                cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
                print "Closest is ", histogram.closest(cv_image, **args)
                if show:
                    histogram.display_histogram(cv_image)
                    if cv2.waitKey(5) == ord('q'):
                        exit(0)
            except CvBridgeError as e:
                self.logger.debug("Error occurred in callback function : %s",e)
                rospy.logerr("Error occurred in callback function : %s",e)
        subscriber = rospy.Subscriber( camera, Image, callback)
    else:
        args["image"] = cv2.imread(arguments["source"])
        print "Closest is ", histogram.closest(**args)
        while True:
            histogram.display_histogram(args["image"])
            if cv2.waitKey(5)==ord('q'):
                exit(0)
        
    rospy.init_node("histogram", anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
       print "Shutting down"
       exit(0)
    cv2.destroyAllWindows()
