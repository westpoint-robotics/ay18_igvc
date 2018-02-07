#import the necessary packages
import cv2
import numpy as np
#'optional' argument is required for trackbar creation parameters
def nothing(x):
    pass
 
#Capture video from the stream
<<<<<<< HEAD
cap = cv2.VideoCapture(0)
=======
cap = cv2.VideoCapture(1)
>>>>>>> 1a8caadf93a9c7ff699ee62b1fec3abefc9c0859
#cap.set(cv2.CAP_PROP_EXPOSURE, 10)

cv2.namedWindow('Colorbars') #Create a window named 'Colorbars'
 
#assign strings for ease of coding
hh='Hue High'
hl='Hue Low'
sh='Saturation High'
sl='Saturation Low'
vh='Value High'
vl='Value Low'
wnd = 'Colorbars'
#Begin Creating trackbars for each
cv2.createTrackbar(hl, 'Colorbars',0,179,nothing)
cv2.createTrackbar(hh, 'Colorbars',0,179,nothing)
cv2.createTrackbar(sl, 'Colorbars',0,255,nothing)
cv2.createTrackbar(sh, 'Colorbars',0,255,nothing)
cv2.createTrackbar(vl, 'Colorbars',0,255,nothing)
cv2.createTrackbar(vh, 'Colorbars',0,255,nothing)

#begin our 'infinite' while loop
while(1):
    #read the streamed frames (we previously named this cap)
    _,frame=cap.read()
 
    #it is common to apply a blur to the frame
    #frame=cv2.GaussianBlur(frame,(5,5),0)
 
    #convert from a BGR stream to an HSV stream
    hsv=cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    #read trackbar positions for each trackbar
    hul=cv2.getTrackbarPos(hl, 'Colorbars')
    huh=cv2.getTrackbarPos(hh, 'Colorbars')
    sal=cv2.getTrackbarPos(sl, 'Colorbars')
    sah=cv2.getTrackbarPos(sh, 'Colorbars')
    val=cv2.getTrackbarPos(vl, 'Colorbars')
    vah=cv2.getTrackbarPos(vh, 'Colorbars')
 
    #make array for final values
    HSVLOW=np.array([hul,sal,val])
    HSVHIGH=np.array([huh,sah,vah])
 
    #create a mask for that range
    mask = cv2.inRange(hsv,HSVLOW, HSVHIGH)

    res = cv2.bitwise_and(frame,frame, mask =mask)
#
<<<<<<< HEAD
    contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

    for index,contour in enumerate(contours):
        x,y,w,h = cv2.boundingRect(contour)
        if(w*h > 100):
            cv2.rectangle(frame,(x,y),(x+w,y+h),(0,0,0),4)
            print x,y
=======
    im2, contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

    for index,contour in enumerate(contours):
        x,y,w,h = cv2.boundingRect(contour)
        cv2.rectangle(frame,(x,y),(x+w,y+h),(0,0,0),4)
>>>>>>> 1a8caadf93a9c7ff699ee62b1fec3abefc9c0859

    #print cap.get(cv2.CAP_PROP_EXPOSURE)
 
    cv2.imshow('res', res)
#
    cv2.imshow('Colorbars', frame)

    if cv2.waitKey(1) & 0xFF == ord('q'): #if CTRL-C is pressed then exit the loop
        break
 
cv2.destroyAllWindows()
