import cv2

'''
Author: Max Krogius

A collection of useful filters, with easy to remember names. These are mostly
aliases for open cv functions
'''

def grayScale(img):
    return cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

def medianBlur(img, size=5):
    return cv2.medianBlur(img, size)

def gaussianBlur(img, size=5):
    return cv2.GaussianBlur(img, (size, size), 0)

def smoothGray(img):
    return grayScale(medianBlur(img))
