import cv2
import numpy as np

def filter(img):
    gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
    gray = cv2.medianBlur(gray,5)

    circles = cv2.HoughCircles(gray,cv2.cv.CV_HOUGH_GRADIENT,2,100,
                                param1=50,param2=85,minRadius=0,maxRadius=0)

    if circles is not None:
        circles = np.round(circles[0, :]).astype("int")
        for i in circles:
            # draw the outer circle
            cv2.circle(img,(i[0],i[1]),i[2],(0,255,0),2)
            # draw the center of the circle
            cv2.circle(img,(i[0],i[1]),2,(0,0,255),3)
    else:
        print "No Circles Detected"

    return img
