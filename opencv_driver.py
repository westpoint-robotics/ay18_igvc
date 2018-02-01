#!/usr/bin/env python

import rospy
from std_msgs.msg import Int16MultiArray

#for opencv
import cv2
import numpy as np;

#read image from camera
cam = cv2.VideoCapture(0)
blue = 1

class Blocks (Structure):
  _fields_ = [ ("type", c_uint),
               ("signature", c_uint),
               ("x", c_uint),
               ("y", c_uint),
               ("width", c_uint),
               ("height", c_uint),
               ("angle", c_uint) ]


def cam_driver():
  pub = rospy.Publisher('cam_signature', Int16MultiArray, queue_size=10)
    rospy.init_node('cam_driver', anonymous=True)
    rate = rospy.Rate(100) # 30hz
    #define array for storing signature data
    sigArray = Int16MultiArray()

    frame  = 0
    while not rospy.is_shutdown():
      ret, view = cam.read()

      #Blurring

      #view = cv2.blur(view, )

      #salt and pepper (grass) correction
      #view = cv2.medianBlur(view, 5)

      #another type
      #view = cv2.bilateralFilter(view, 9, 75, 75)

      # ret is true/false
      if not ret:
        break

      #convert from RGB to HSV
      hsv = cv2.cvtColor(view, cv2.COLOR_BGR2HSV)

      canvas = view.copy
      #redLower = (0,0,150)
      #redUpper = (50,50,255)

      #(hue, saturation, value)
      blueLower = np.array([110, 50, 50])
      blueUpper = np.array([130, 255, 255])
      blueMask = cv2.inRange(hsv, blueLower, blueUpper)

      #Bitwise and maskwise and original image
      res = cv2.bitwise_and(view, view, mask = blueMask)
      #blueMask = blueMask?

      #find new colors with 
        #green = np.uint8([[[0,255,0 ]]])
        #hsv_green = cv2.cvtColor(green, cv2.COLOR_BGR2HSV)
        #print hsv_green
        #[[[ 60 255 255]]]
        #lower = [hue - 10, 100, 100] and higher = [hue + 10, 255, 255]

      im2, contours, hierarchy = cv2.findContours(blueMask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
      if contours: #empty list should be false
        frame = frame + 1
        for index, contour in enumerate(contours):
          x,y,w,h = cv2.boundingRect(contour)
          cv2.rectangle(canvas, (x, y), (x+w, y+h), (0,0,0), 4)
          blockInfo = [frame, blue, x, y, w, h, 0]
          sigArray.data = blockInfo
          pub.publish(sigArray)
      else:
          sigArray.data = [0,0,0,0,0,0,0]
          pub.publish(sigArray)
      rate.sleep()

if __name__ == '__main__':
    try:
        cam_driver()
    except rospy.ROSInterruptException:
        pass