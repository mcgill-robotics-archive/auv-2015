import cv2
import numpy as np
import math

'''
Author: Max Krogius

A collection of useful filters, with easy to remember names. These are mostly
aliases for open cv functions
'''


def grayScale(img):
    if len(img.shape) == 3:
        return cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    return img


def medianBlur(img, size=5):
    return cv2.medianBlur(img, size)


def gaussianBlur(img, size=5):
    return cv2.GaussianBlur(img, (size, size), 0)


def smoothGray(img):
    return grayScale(medianBlur(img))


def filterSize(contours, min_length, min_area):
    # Removes small contours
    return [c for c in contours
            if cv2.arcLength(c, closed=True) > min_length
            and cv2.contourArea(c) > min_area]


def suppress_bad_shapes(contours, shape, threshold):
    return [c for c in contours
            if cv2.matchShapes(c, shape, 3, 0.0) < threshold]


def dist(p1, p2):
    # Distance between two points
    return math.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)


def contourIntensityAverage(img, outerContour, innerContour=None):
    # Calculates the average intensity inside outerContour and outside
    # innerContour (if provided)
    mask = np.zeros(img.shape, np.uint8)
    cv2.drawContours(mask, [outerContour], 0, 255, -1)
    if innerContour is not None:
        cv2.drawContours(mask, [innerContour], 0, 0, -1)
    return cv2.mean(img, mask=mask)[0]


def filter_intensities(contours, img, flag, intensity_change):
    # Tests for rectangles where the area just outside the border
    # is lighter than just inside. This is useful to suppress the
    # outside of the bins.
    if flag == 0:
        return contours

    valid = []
    border = 0.1
    for c in contours:
        # Scale contour up and down
        outer_border = np.int32((1+border)*c - border*np.average(c, axis=0))
        inner_border = np.int32((1-border)*c + border*np.average(c, axis=0))
        # Calculate average inner and outer border intensities
        outer_avg = contourIntensityAverage(img, outer_border, c)
        inner_avg = contourIntensityAverage(img, c, inner_border)
        if flag == 1:
            if outer_avg > inner_avg + intensity_change:
                valid.append(c)
        elif flag == -1:
            if inner_avg > outer_avg + intensity_change:
                valid.append(c)
    return valid


def suppress_concentric(contours):
    # Suppresses concentric contours by looking at their enclosing circle
    valid = []
    for x in contours:
        x_center, x_rad = cv2.minEnclosingCircle(x)
        for y in contours:
            y_center, y_rad = cv2.minEnclosingCircle(y)
            if y_rad > x_rad and dist(x_center, y_center) < y_rad:
                break
        else:
            valid.append(x)
    return valid


COLORS = ((0, 255, 0),  # Green
          (0, 255, 255),  # Yellow
          (0, 140, 255),  # Dark Orange
          (0, 0, 255),  # Red
          (240, 32, 160),  # Purple
          (255, 0, 0))  # Blue


def validate_contours(contours, filter_list, debug):
    for i, f in enumerate(filter_list):
        contours = f(contours)
        if len(filter_list) - i - 1 < len(COLORS):
            cv2.drawContours(debug, contours, -1,
                             COLORS[len(filter_list) - i - 1], 3)
    return contours, debug
