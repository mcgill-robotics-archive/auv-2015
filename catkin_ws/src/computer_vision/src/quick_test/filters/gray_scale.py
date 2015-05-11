import cv2

def filter(img):
    return cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
