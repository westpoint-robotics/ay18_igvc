#!/usr/bin/env python

import rospy
from std_msgs.msg import Int16MultiArray

#for opencv
import cv2
import numpy as np;

#read image from camera
cam = cv2.VideoCapture(0)
red = 1

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

      if not ret:
        break

      canvas = view.copy
      redLower = (0,0,150)
      redUpper = (50,50,255)
      redMask = cv2.inRange(view, redLower, redUpper)
      im2, contours, hierarchy = cv2.findContours(redMask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
      if contours: #empty list should be false
        frame = frame + 1
        for index, contour in enumerate(contours):
          x,y,w,h = cv2.boundingRect(contour)
          cv2.rectangle(canvas, (x, y), (x+w, y+h), (0,0,0), 4)
          blockInfo = [frame, red, x, y, w, h, 0]
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