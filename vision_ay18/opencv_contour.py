import cv2
import numpy as np;

# Uses opencv to create bounding boxes around countours of blobs of designated colors
# Written by: MAJ Dominic Larkin
# Date: 22JAN2018

# Read image from the camera
cam = cv2.VideoCapture(0) # This is typically either 0 or 1, if your computer has a built in camera.

while(1):
        ret, frame = cam.read()

        if not ret: # if camera is not found then exit program
            break
        #Contours reference: https://docs.opencv.org/3.4.0/dd/d49/tutorial_py_contour_features.html
        canvas = frame.copy() # create a copy of the frame
        lower = (0,0,150)  # find red
        upper = (50,50,255) # Find red
        mask = cv2.inRange(frame, lower, upper)
        im2, contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        for index,contour in enumerate(contours):
            x,y,w,h = cv2.boundingRect(contour)
            cv2.rectangle(canvas,(x,y),(x+w,y+h),(0,0,0),4)
            #print(x,y,w,h)
        #print("Done with this iteration")

        #cv2.imshow('frame',frame) # original image
        cv2.imshow('canvas',canvas) # canvas with with bounding box around contour
        cv2.imshow('mask',im2) # returns 8-bit single-channel image 0 or 255 as possible values. 0 represents found pixel.

        if cv2.waitKey(1) & 0xFF == ord('q'): #if CTRL-C is pressed then exit the loop
            break

cv2.destroyAllWindows()
