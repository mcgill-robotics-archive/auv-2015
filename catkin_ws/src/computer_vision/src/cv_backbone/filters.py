import cv2

'''
Author: Max Krogius

A collection of useful filters, with easy to remember names. These are mostly
aliases for open cv functions
'''

def grayScale(img):
    if len(img.shape)==3:
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
            
def suppressBadShapes(contours, shape, threshold):
    return [c for c in contours 
            if cv2.matchShapes(c, shape, 3, 0.0) < threshold]
