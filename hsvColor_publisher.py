#import the necessary packages
import rospy
from std_msgs.msg import Int16MultiArray
from sensor_msgs.msg import Image
import cv2, math
import numpy as np
from cv_bridge import CvBridge
cv_image = np.zeros((1280,960,3), np.uint8)
res = np.zeros((1280,960,3), np.uint8)
res2 = np.zeros((1280,960,3), np.uint8)
cropY = 300
sigArray = Int16MultiArray()
sigArray.data = [0,0,0,0,0,0,0]
finalSignature = [0,0,0,0,0,0,0]
pub = rospy.Publisher('centerLine',Int16MultiArray, queue_size=10)

obstacleXLeftIgnore = 200
lineXLeftIgnore = 300
lineYLeftIgnore = 500

#'optional' argument is required for trackbar creation parameters
def nothing(x):
    pass

def mouseClick(event,x,y,flags,param):
    global mouseX, mouseY
    if event == cv2.EVENT_LBUTTONDBLCLK:
        print x,y
cv2.namedWindow('Camera')
cv2.setMouseCallback('Camera',mouseClick)
def callback(data):
    global cv_image
    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")
    #read trackbar positions for each trackbar
    

rospy.init_node('pointgrey_mine', anonymous=True)
rospy.Subscriber("/camera/image_color",Image, callback)
 
#Capture video from the stream
cap = cv2.VideoCapture(0)
#cap.set(cv2.CAP_PROP_EXPOSURE, 10)

cv2.namedWindow('Colorbars') #Create a window named 'Colorbars'
cv2.namedWindow('Colorbars2')
 
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
cv2.createTrackbar("Noise", 'Colorbars',0,15000,nothing)

cv2.createTrackbar(hl, 'Colorbars2',0,179,nothing)
cv2.createTrackbar(hh, 'Colorbars2',0,179,nothing)
cv2.createTrackbar(sl, 'Colorbars2',0,255,nothing)
cv2.createTrackbar(sh, 'Colorbars2',0,255,nothing)
cv2.createTrackbar(vl, 'Colorbars2',0,255,nothing)
cv2.createTrackbar(vh, 'Colorbars2',0,255,nothing)

rate = rospy.Rate(10) # 10hz
frameNumber = 0
#begin our 'infinite' while loop
while not rospy.is_shutdown(): 
    #it is common to apply a blur to the cv_image
    #cv_image=cv2.GaussianBlur(cv_image,(5,5),0)
    #convert from a BGR stream to an HSV stream
    cv_imageBlurred=cv2.GaussianBlur(cv_image,(5,5),0)
    hsv=cv2.cvtColor(cv_imageBlurred, cv2.COLOR_BGR2HSV)
    k = cv2.waitKey(20) & 0xFF
    finalSignature = [0,0,0,0,0,0,0]
    if k == 27:
        break
    elif k == ord('a'):
        print mouseX,mouseY

    hul=cv2.getTrackbarPos(hl, 'Colorbars')
    huh=cv2.getTrackbarPos(hh, 'Colorbars')
    sal=cv2.getTrackbarPos(sl, 'Colorbars')
    sah=cv2.getTrackbarPos(sh, 'Colorbars')
    val=cv2.getTrackbarPos(vl, 'Colorbars')
    vah=cv2.getTrackbarPos(vh, 'Colorbars')
    minNoise = cv2.getTrackbarPos("Noise", 'Colorbars')

    hul2=cv2.getTrackbarPos(hl, 'Colorbars2')
    huh2=cv2.getTrackbarPos(hh, 'Colorbars2')
    sal2=cv2.getTrackbarPos(sl, 'Colorbars2')
    sah2=cv2.getTrackbarPos(sh, 'Colorbars2')
    val2=cv2.getTrackbarPos(vl, 'Colorbars2')
    vah2=cv2.getTrackbarPos(vh, 'Colorbars2')
 
    #make array for final values
    HSVLOW=np.array([hul,sal,val])
    HSVHIGH=np.array([huh,sah,vah])

    HSVLOW2=np.array([hul2,sal2,val2])
    HSVHIGH2=np.array([huh2,sah2,vah2])  
 
    #create a mask for that range
    mask = cv2.inRange(hsv,HSVLOW, HSVHIGH)
    mask2 = cv2.inRange(hsv,HSVLOW2, HSVHIGH2)
    mask = mask[cropY:964,0:1288]
    mask2 = mask2[cropY:964,0:1288]
    cv_imageBlurred2 = cv_imageBlurred[cropY:964,0:1288]

    res = cv2.bitwise_and(cv_imageBlurred2,cv_imageBlurred2, mask = mask)
    res2 = cv2.bitwise_and(cv_imageBlurred2,cv_imageBlurred2, mask = mask2)
    contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    contours2, hierarchy2 = cv2.findContours(mask2, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

    if contours or contours2:
      detect_flag = False
      frameNumber += 1
      sigNum = 0
      for index,contour in enumerate(contours):
        sigNum += 1
        rect = cv2.minAreaRect(contour)
        #print rect[0][0] - rect[1][1]/2 , rect[0][1]+cropY
        box = cv2.cv.BoxPoints(rect)
        box = np.int0(box)
        for z in range(4):
          box[z][1] += cropY
        if rect[1][0] * rect[1][1] > minNoise:          
          sigX,sigY = rect[0][0] - math.cos(math.radians(rect[2]))*rect[1][1]/2, rect[0][1]+cropY + math.sin(math.radians(rect[2]))*rect[1][1]/2

          if not (sigY < lineYLeftIgnore and sigX < lineXLeftIgnore):
            detect_flag = True
            finalSignature = [frameNumber,1,sigNum,rect[0][0],rect[0][1]+cropY,rect[1][1]/2,rect[2]]
            cv2.drawContours(cv_imageBlurred, [box], 0, (0,255,0), 2)       

      for index, contour in enumerate(contours2):
        sigNum += 1
        x,y,w,h = cv2.boundingRect(contour)
        y += cropY
        if(w*h > 10000):
          sigX,sigY = x, y + h
          detect_flag = True
          cv2.rectangle(cv_imageBlurred, (x, y), (x+w, y+h), (0,0,0), 4)
          if sigX + w > obstacleXLeftIgnore:
            if (finalSignature[1] > 1 and finalSignature[4] < sigY) or finalSignature[1] <= 1:
              finalSignature = [frameNumber, 3, sigNum, sigX, sigY, w, h]
              
              #cv2.drawContours(cv_imageBlurred, contour, -1, (0,255,0), 3)



      for index,contour in enumerate(contours):
        rect = cv2.minAreaRect(contour)
        (center, (w, h), angle) = rect

        epsilon = 0.1*cv2.arcLength(contour,True)
        approx = cv2.approxPolyDP(contour,epsilon,True)
        #if(w*h >= 10000):
           #print(frameNumber)
           #cv2.drawContours(cv_imageBlurred, contour, -1, (0,255,0), 3)

      if not detect_flag:
            sigArray.data = [0,0,0,0,0,0,0]
            pub.publish(sigArray)
    else:
        sigArray.data = [0,0,0,0,0,0,0]
        pub.publish(sigArray)
    sigArray.data=finalSignature
    pub.publish(sigArray)
    cv2.imshow('Camera', cv_imageBlurred)

 
    cv2.imshow('res', res)
    cv2.imshow('res2', res2)

    if cv2.waitKey(1) & 0xFF == ord('q'): #if CTRL-C is pressed then exit the loop
        break
    rate.sleep()
cv2.destroyAllWindows()
