#import the necessary packages
import cv2
import numpy as np
#'optional' argument is required for trackbar creation parameters
def nothing(x):
    pass
 
#Capture video from the stream
cap = cv2.VideoCapture(1)
#cap.set(cv2.CAP_PROP_EXPOSURE, 10)

cv2.namedWindow('Colorbars') #Create a window named 'Colorbars'

#begin our 'infinite' while loop
frameNumber = 0
while(1):
    #read the streamed frames (we previously named this cap)
    _,frame=cap.read()
 
    #it is common to apply a blur to the frame
    frame=cv2.GaussianBlur(frame,(5,5),0)
 
    #convert from a BGR stream to an HSV stream
    hsv=cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
 
    #make array for final values
    HSVLOW=np.array([98,79,187])
    HSVHIGH=np.array([130,255,255])
 
    #create a mask for that range
    mask = cv2.inRange(hsv,HSVLOW, HSVHIGH)

    res = cv2.bitwise_and(frame,frame, mask =mask)
#
    im2, contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

    if contours:
        frameNumber = frameNumber + 1

        for index,contour in enumerate(contours):
            x,y,w,h = cv2.boundingRect(contour)
            cv2.rectangle(frame,(x,y),(x+w,y+h),(0,0,0),4)
            print frameNumber, x,y,w,h
 
    cv2.imshow('res', res)
#
    cv2.imshow('Colorbars', frame)

    if cv2.waitKey(1) & 0xFF == ord('q'): #if CTRL-C is pressed then exit the loop
        break
 
cv2.destroyAllWindows()
