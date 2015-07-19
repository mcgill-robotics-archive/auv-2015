#! /usr/bin/env python2.7

import cv2
import math
import numpy as np
import glob
from cv_backbone import CvBackbone, filters

'''
Author: Max Krogius

The approach used here is to use findContours, and then filter out bad
candidates based on size and shape. We are searching here for the inner
edge of the bin, as the outer edges may be touching and so may not be
identified.
'''

silhouettes = None


def filterSize(contours, min_length, min_area):
    # Removes small contours
    return [c for c in contours
            if cv2.arcLength(c, closed=True) > min_length
            and cv2.contourArea(c) > min_area]


def suppressBadShapes(contours):
    # This works well without skew, but with skew it doesn't work so well.
    # It allows things that look like rectanges but with more vertices.
    # In theory that is good, but in the end we do need rectangles for
    # perspective transform.
    # This also applies the constraint that the rectangle we want is about
    # twice as long in one direction
    rect = np.array([[[0, 0]], [[0, 1]], [[2, 1]], [[2, 0]]])
    return [c for c in contours if cv2.matchShapes(c, rect, 1, 0.0) < 0.1]


def filterRectangles(contours, tol):
    # This suppresses some good shapes, why? Because they have extra vertices.
    # Increasing the tolerance causes more stuff to be fit as quadrilaterals.
    valid = []
    for c in contours:
        poly = cv2.approxPolyDP(c, tol*cv2.arcLength(c, True), True)
        if len(poly) == 4 and cv2.isContourConvex(poly):
            valid.append(poly)
    return valid


def dist(p1, p2):
    # Distance between two points
    return math.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)


def sideLengths(rect):
    # Returns the lengths of the sides of the rectangle
    return map(lambda i: dist(rect[i-1], rect[i]), range(4))


def filterAspectRatio(rects, desired_aspect_ratio, aspect_ratio_tol):
    # Test that the aspect ratio of the rectangle i.e is it x times
    # as long in one direction as the other.
    desired_aspect_ratio = desired_aspect_ratio if desired_aspect_ratio > 1 \
        else 1/desired_aspect_ratio
    valid = []
    for r in rects:
        r = r.reshape(-1, 2)
        lengths = sideLengths(r)
        aspect_ratio = (lengths[0] + lengths[2])/(lengths[1] + lengths[3])
        aspect_ratio = aspect_ratio if aspect_ratio > 1 else 1/aspect_ratio
        if abs(aspect_ratio - desired_aspect_ratio) < aspect_ratio_tol:
            valid.append(r)
    return valid


def contourIntensityAverage(img, outerContour, innerContour=None):
    # Calculates the average intensity inside outerContour and outside
    # innerContour (if provided)
    mask = np.zeros(img.shape, np.uint8)
    cv2.drawContours(mask, [outerContour], 0, 255, -1)
    if innerContour is not None:
        cv2.drawContours(mask, [innerContour], 0, 0, -1)
    return cv2.mean(img, mask=mask)[0]


def filterIntensities(contours, img, flag, intensity_change):
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


def suppressConcentric(contours):
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


def angle_cos(p0, p1, p2):
    d1, d2 = (p0-p1).astype('float'), (p2-p1).astype('float')
    return abs(np.dot(d1, d2) / np.sqrt(np.dot(d1, d1)*np.dot(d2, d2)))


def suppressSkewed(contours, cos_thresh):
    valid = []
    for c in contours:
        coses = [angle_cos(c[i-2], c[i-1], c[i]) for i in range(4)]
        max_cos = np.max(coses)
        if max_cos < cos_thresh:
            valid.append(c)
    return valid


def validate(contours, img):
    # Minimum perimeter of target
    min_length = 4*20
    # Minimum area of target
    min_area = 20*20
    validContours = filterSize(contours, min_length, min_area)
    # Tolerance for polygon fitting. Increasing means things get fit
    # with fewer vertices.
    polygon_approx_tolerance = 0.05
    validContours = filterRectangles(validContours, polygon_approx_tolerance)
    # The ratio of size in one dimension to the other.
    aspect_ratio = 2
    # Allowed variance in aspect ratio
    aspect_ratio_tol = 0.5
    validContours = \
        filterAspectRatio(validContours, aspect_ratio, aspect_ratio_tol)
    # Flag for how the intensity changes from the inside of the border to the
    # outside of the border. 1 means that the outside is more intense than the
    # inside. 0 means there isn't a difference. -1 means the inside is more
    # intense than the outside.
    intensity_change_flag = 1
    # Minimum average increase in intensity
    intensity_change = 10
    validContours = filterIntensities(validContours, img,
                                      intensity_change_flag, intensity_change)
    validContours = suppressConcentric(validContours)
    # Max cosine of any internal angle of the rectangle
    max_cos = 0.15
    validContours = suppressSkewed(validContours, max_cos)
    return validContours


def matchShapes(img1, img2):
    # Approach is to threshold and compare

    # Otsu threshold
    thrsh11 = cb.filters.gaussianBlur(img1)
    _, thrsh11 = cv2.threshold(thrsh11, 0, 255,
                               cv2.THRESH_OTSU + cv2.THRESH_BINARY)

    # Adaptive gaussian threshold
    thrsh12 = cb.filters.gaussianBlur(img1.copy())
    thrsh12 = cv2.adaptiveThreshold(
        thrsh12, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 11, 2)

    # Combine the two. Otsu gets the internal details and adaptive threshold
    # gets the general shape.
    thrsh1 = cv2.bitwise_and(thrsh11, thrsh12)

    # Remove any white around the edges. The silhouette won't reach that far
    thickness = 10
    cv2.rectangle(thrsh1, (0, 0), (200, thickness),
                  color=0, thickness=cv2.cv.CV_FILLED)
    cv2.rectangle(thrsh1, (0, 0), (thickness, 100),
                  color=0, thickness=cv2.cv.CV_FILLED)
    cv2.rectangle(thrsh1, (200, 100), (200 - thickness, 0),
                  color=0, thickness=cv2.cv.CV_FILLED)
    cv2.rectangle(thrsh1, (200, 100), (0, 100 - thickness),
                  color=0, thickness=cv2.cv.CV_FILLED)

    # Threshold the silhouette. Even if it looks black and white in your image
    # editor, it is not necessarily binary
    _, thrsh2 = cv2.threshold(img2, 0, 255,
                              cv2.THRESH_OTSU + cv2.THRESH_BINARY)
    huMoments1 = cv2.HuMoments(cv2.moments(thrsh1, binaryImage=True))
    huMoments2 = cv2.HuMoments(cv2.moments(thrsh2, binaryImage=True))
    m1 = np.sign(huMoments1) * np.log(np.abs(huMoments1))
    m2 = np.sign(huMoments2) * np.log(np.abs(huMoments2))
    return np.sum(np.abs(m1 - m2))


def matchSilhouettes(img, rects, orig):
    src_rect = 100.0 * np.array([[0, 0], [0, 1], [2, 1], [2, 0]])
    for rect in rects:
        # Ensure that the rectangle has the long side matched with
        # the long side of src_rect
        lengths = sideLengths(rect)
        aspect_ratio = (lengths[0] + lengths[2])/(lengths[1] + lengths[3])
        if aspect_ratio < 1:
            rect = np.roll(rect, 1, axis=0)
        transform = cv2.getPerspectiveTransform(np.float32(rect),
                                                np.float32(src_rect))
        warped = cv2.warpPerspective(img, transform, (200, 100))

        # Choose the silhouettes which is the closest match
        name = 'unknown'
        min_dist = float('inf')
        for sil_name, sil in silhouettes.iteritems():
            # TODO: Do we need copies here? Also the silhouette moments
            # can be pre-computed. Also we should look at fourier descriptors.
            dist = matchShapes(warped.copy(), sil.copy())
            if dist < min_dist:
                min_dist = dist
                name = sil_name
        cv2.putText(orig, name, tuple(np.int32(np.average(rect, axis=0))),
                    cv2.FONT_HERSHEY_PLAIN, 1, (0, 0, 255))


def findByContours(orig):
    img = orig.copy()
    gray = filters.grayScale(img)
    gray = filters.medianBlur(gray)
    img = cv2.Canny(gray, 0, 100, apertureSize=5)
    contours, _ = cv2.findContours(img, cv2.RETR_LIST, cv2.CHAIN_APPROX_SIMPLE)
    rects = validate(contours, gray)
    return rects


def debug_callback(orig):
    rects = findByContours(orig)
    matchSilhouettes(cv2.split(orig)[2], rects, orig)
    cv2.drawContours(orig, rects, -1, (0, 0, 255), 1)
    cb.publishImage(orig)


def loadSilhouettes():
    global silhouettes
    silhouettes = {}
    try:
        for file_name in glob.glob(cb.get_param('~silhouettesPath')):
            img = cv2.imread(file_name, cv2.CV_LOAD_IMAGE_GRAYSCALE)
            name = file_name.split('.')[0]
            name = name.split('/')[-1]
            silhouettes[name] = img
    except:
        print('Could not load silhouettes')






if __name__ == '__main__':
    global cb
    cb = CvBackbone.CvBackbone('bin_detection')
    cb.addImageCallback(debug_callback)
    cb.onStart(loadSilhouettes)
    cb.start()
