#!/usr/bin/env python
#rosrun  image_cv_image image_cv_image image:=/camera/image_color

import rospy
from std_msgs.msg import Int16MultiArray
from ctypes import *
from sensor_msgs.msg import Image

#for opencv
import cv2
import numpy as np;
from cv_bridge import CvBridge
cv_image = np.zeros((640,480,3), np.uint8)

sigArray = Int16MultiArray()
sigArray.data = [0,0,0,0,0,0,0]
frame = 0
#read image from camera
cam = cv2.VideoCapture(0)
blue = 1
cv2.namedWindow('Camera') 
pub = rospy.Publisher('camera_signature', Int16MultiArray, queue_size=10)

def callback(data):
    global cv_image
    global frame
    global sigArray
    global pub
    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(data, desired_encoding="rgb8")
    cv_image=cv2.GaussianBlur(cv_image,(5,5),0)

      #convert from RGB to HSV
    hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

    canvas = cv_image.copy
      #redLower = (0,0,150)
      #redUpper = (50,50,255)

      #(hue, saturation, value)
    whiteLower = np.array([0, 0, 220])
    whiteUpper = np.array([10, 10, 255])
    whiteMask = cv2.inRange(hsv, whiteLower, whiteUpper)
    whiteMask = whiteMask[575:964,0:1284]
    #cv_image2 = cv_image[575:964,0:1284]

    blueLower = np.array([96, 89, 187])
    blueUpper = np.array([145, 184, 241])
    blueMask = cv2.inRange(hsv, blueLower, blueUpper)
    blueMask = = blueMask[575:964,0:1284]

      #Bitwise and maskwise and original image
    #res = cv2.bitwise_and(cv_image, cv_image, mask = whiteMask)
      #blueMask = blueMask?

      #find new colors with 
        #green = np.uint8([[[0,255,0 ]]])
        #hsv_green = cv2.cvtColor(green, cv2.COLOR_BGR2HSV)
        #print hsv_green
        #[[[ 60 255 255]]]
        #lower = [hue - 10, 100, 100] and higher = [hue + 10, 255, 255]

    contours, hierarchy = cv2.findContours(whiteMask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    contours2, hierarchy2 = cv2.findContours(blueMask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
      
    if contours: #empty list should be false
        frame = frame + 1
        for index,contour in enumerate(contours):
            rect = cv2.minAreaRect(contour)
            #print rect[0][0] - rect[1][1]/2
            if rect[1][0] * rect[1][1] > 500:
                blockInfo = [frame,1,0,rect[0][0],rect[0][1]+575,rect[1][1]/2,rect[3]]
                sigArray.data = blockInfo
                pub.publish(sigArray)
        for index, contour in enumerate(contours2):
            x,y,w,h = cv2.boundingRect(contour)
            if(w*h > 10000):
                blockInfo = [frame, 2, 0, x, y+575, w, h]
                sigArray.data = blockInfo
                pub.publish(sigArray)
        #for index, contour in enumerate(contours):
         #   x,y,w,h = cv2.boundingRect(contour)
          #  if(w*h > 10000):
           #     cv2.rectangle(cv_image, (x, y), (x+w, y+h), (0,0,0), 4)
            #    blockInfo = [frame, blue, 0, x, y, w, h]
             #   sigArray.data = blockInfo
              #  pub.publish(sigArray)
                    #cv2.imshow('Camera', cv_image)
                    #print frame, w*h
    else:
        #cv2.imshow('Camera', cv_image)
        sigArray.data = [0,0,0,0,0,0,0]
        pub.publish(sigArray)

class Blocks (Structure):
  _fields_ = [ ("type", c_uint),
               ("signature", c_uint),
               ("x", c_uint),
               ("y", c_uint),
               ("width", c_uint),
               ("height", c_uint),
               ("angle", c_uint) ]


def camera_driver():
    rospy.init_node('camera_driver', anonymous=True)
    rospy.Subscriber("/camera/image_color",Image, callback)
    rate = rospy.Rate(100) # 30hz
    #define array for storing signature data
    
    frames = 0
    while not rospy.is_shutdown():
        frames += 1
        #cv2.imshow('res', res)
        #ret, cv_image = cam.read()

      #Blurring

      #cv_image = cv2.blur(cv_image, )

      #salt and pepper (grass) correction
      #cv_image = cv2.medianBlur(cv_image, 5)

      #another type
      #cv_image = cv2.bilateralFilter(cv_image, 9, 75, 75)

      # ret is true/false
        
      
    rate.sleep()

if __name__ == '__main__':
    try:
        camera_driver()
    except rospy.ROSInterruptException:
        pass

