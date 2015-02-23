#!/usr/bin/env python
import cv2
import argparse       
import numpy as np

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="Take preprocessed image files to train machine")
    parser.add_argument("x", help="File path to list of feature vectors x")
    parser.add_argument("y", help="File path to supervisory signals y")
    arguments = vars(parser.parse_args())
    x = np.loadtxt(arguments["x"],np.float32)
    y = np.loadtxt(arguments["y"],np.float32)
    y = y.reshape((y.size, 1))
    model = cv2.KNearest(x,y)
    ## nuclear test zone
    test = cv2.imread("../Pictures/numberset1.jpeg")
    out = np.zeros( test.shape, np.uint8)
    thresh = cv2.adaptiveThreshold( cv2.cvtColor(test, cv2.COLOR_BGR2GRAY), 255,1,1,11,2)
    contours,hierarchy = cv2.findContours(thresh,cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)
    for cnt in contours:
        if cv2.contourArea(cnt)>50:
            x,y,w,h = cv2.boundingRect(cnt)
            if  h>=64 and w>=64:
                cv2.rectangle(test,(x,y),(x+w,y+h),(0,255,0),2)
                roi = thresh[y:y+h,x:x+w]
                roismall = cv2.resize(roi,(64 ,64))
                roismall = roismall.reshape((1,64*64))
                roismall = np.float32(roismall)
                retval, results, neigh_resp, dists = model.find_nearest(roismall, k = 1)
                string = str(int((results[0][0])))
                cv2.putText(out,string,(x,y+h),0,1,(0,255,0))
    cv2.imshow("image" , test)
    cv2.imshow("out", out)
    if cv2.waitKey() == ord('q'):
        exit(0)
    
