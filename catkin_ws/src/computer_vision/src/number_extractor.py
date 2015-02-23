#!/usr/bin/env python
import cv2
import argparse       
import numpy as np

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="""Takes an image of character set and extract the characters to individual images.
                                                    Note: characters in the image set should be larger than 64 in both dimensions
                                                    and the characters should preferably be black on white background. The output images are 64x64""")
    parser.add_argument("src", help="Source file. The character set.")
    parser.add_argument("dst", help="Destination folder")
    parser.add_argument("-p", "--prefix", help="Prepend prefix to output files")
    parser.add_argument("-v", "--verbose", help="Give more information about what is going on", action="store_true")
    arguments = vars(parser.parse_args())
    path = arguments["src"]
    image = cv2.imread(path)
    # process the image first
    image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    ret,thresh = cv2.threshold(image,1,255,0)
    thresh = cv2.bitwise_not(thresh)
    contours, hierarchy = cv2.findContours(thresh,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    dst = arguments["dst"]
    prefix = arguments["prefix"]+"_" if arguments["prefix"] else ""
    verbose = arguments["verbose"]
    # setup constants
    roi_size = 64
    for i,cnt in enumerate(contours):
        x,y,w,h = cv2.boundingRect(cnt)
        # get boxes which are significant
        if verbose:
            print "box", i, "is", w, "wide", h, "high"
        if h>=64 and w>=64:
            roi = image[y:y+h,x:x+w]
            resized = cv2.resize(roi, (roi_size,roi_size))  
            cv2.imwrite(dst+prefix+str(i)+".png", resized)
