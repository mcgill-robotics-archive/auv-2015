#! /usr/bin/env python2.7

import cv2
import numpy as np
from numpy import linalg as LA
from cv_backbone import CvBackbone, filters

'''
Author: Wei-Di Chang

The approach used here is to use findContours, and then filter out bad
candidates based on size and shape.
'''


def suppress_bad_shapes(contours):
    # Here we try to find something that looks like a lane, including the legs
    lane = np.array([[[5, 0]], [[5, 14]], [[0, 14]], [[0, 16]],
                    [[5, 16]], [[5, 70]], [[0, 70]], [[0, 72]],
                    [[5, 72]], [[5, 86]], [[15, 86]], [[15, 72]],
                    [[20, 72]], [[20, 70]], [[15, 70]], [[15, 16]],
                    [[20, 16]], [[20, 14]], [[15, 14]], [[15, 0]]])
    return filters.suppress_bad_shapes(contours, lane, 1.25)


def angle_cos(p0, p1, p2):
    d1, d2 = (p0-p1).astype('float'), (p2-p1).astype('float')
    return abs(np.dot(d1, d2) / np.sqrt(np.dot(d1, d1)*np.dot(d2, d2)))


def filterEccentricity(contours, eccentricity_thresh):
    # Filters out contours based on ratio of major-axis/minor-axis - not
    # exactly "eccentricity" by definition
    valid = []
    for cnt in contours:
        moment = cv2.moments(cnt)
        if(moment['m00'] != 0):
            covariance = np.matrix([[moment['m20']/moment['m00'],
                                   moment['m11']/moment['m00']],
                                   [moment['m11']/moment['m00'],
                                   moment['m02']/moment['m00']]])
        else:
            continue
        eigenValues = LA.eigvals(covariance)
        eigenValues.sort()
        eccentricity = np.sqrt(eigenValues[1]/eigenValues[0])
        # print eccentricity
        if(eccentricity < eccentricity_thresh):
            valid.append(cnt)
    return valid


def validate(contours, img, debug):
    filter_list = []
    # Minimum perimeter of target
    min_length = 20*4
    # Minimum area of target
    min_area = 35  # Increasing to 40 suppresses target
    filter_list.append(lambda x: filters.filterSize(x, min_length, min_area))
    # Tolerance for polygon fitting. Increasing means things get fit
    # with fewer vertices.
    # eccentricity_threshold = 17
    # filter_list.append(filterEccentricity(contours, eccentricity_threshold))
    filter_list.append(lambda x: suppress_bad_shapes(x))

    # Flag for how the intensity changes from the inside of the border to the
    # outside of the border. 1 means that the outside is more intense than the
    # inside. 0 means there isn't a difference. -1 means the inside is more
    # intense than the outside.
    intensity_change_flag = -1
    # Minimum average increase in intensity
    # For Lanes, inside is more intense in the Red channel by about
    # 115 (= 175 - 60) Using greyscale for now, inside more intense by 70
    intensity_change = 20
    filter_list.append(lambda x: filters.filter_intensities(x, img,
                       intensity_change_flag, intensity_change))
    filter_list.append(lambda x: filters.suppress_concentric(x))

    return filters.validate_contours(contours, filter_list, debug)


def findByContours(orig):
    img = orig.copy()
    debug = orig.copy()
    gray = cv2.split(img)[2]  # cb.filters.grayScale(img)
    gray = cb.filters.gaussianBlur(gray)
    img = cv2.Canny(gray, 0, 100, apertureSize=5)
    contours, _ = cv2.findContours(img, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    return validate(contours, cv2.split(orig)[2], debug)


def debug_callback(orig):
    #gray = cb.filters.grayScale(orig)
    #gray = cv2.split(orig)[2]
    #gray = cb.filters.medianBlur(gray)
    #img = cv2.Canny(gray, 0, 100, apertureSize=5)
    _, debug = findByContours(orig)
    cb.publishImage(debug)

if __name__ == '__main__':
    global cb
    cb = CvBackbone.CvBackbone('lane_detection')
    cb.addImageCallback(debug_callback)
    cb.start()
