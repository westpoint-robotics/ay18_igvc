#import the necessary packages
import rospy
from std_msgs.msg import Int16MultiArray
from sensor_msgs.msg import Image
import cv2, math
import numpy as np
from cv_bridge import CvBridge
cv_image = np.zeros((1288,964,3), np.uint8)
res = np.zeros((1288,964,3), np.uint8)
res2 = np.zeros((1288,964,3), np.uint8)
cropY = 0
cutOffY = 300
sigArray = Int16MultiArray()
sigArray.data = [0,0,0,0,0,0,0]
pub = rospy.Publisher('centerLine',Int16MultiArray, queue_size=1)

def nothing(x):
    pass

def callback(data):
    global cv_image
    bridge = CvBridge()
    cv_image = bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")

rospy.init_node('pointgrey_mine', anonymous=True)
rospy.Subscriber("/camera/image_color",Image, callback)

rate = rospy.Rate(30)
frameNumber = 0

obstacleXLeftIgnore = 200
lineXLeftIgnore = 300
lineYLeftIgnore = 500

minLineNoise = 9000
minObstacleNoise = 10000

#WHITE
hul=0
huh=179
sal=0
sah=30
val=0
vah=255

#WHITE
hul0=64
huh0=179
sal0=0
sah0=255
val0=200
vah0=199

#GREEN
hul1=67
huh1=112
sal1=0
sah1=255
val1=100
vah1=145

#YELLOW
hul2=15
huh2=30
sal2=171
sah2=255
val2=50
vah2=255

#ORANGE
hul3=0
huh3=15
sal3=80
sah3=255
val3=100
vah3=255

#BLUE
hul4=107
huh4=130
sal4=129
sah4=255
val4=104
vah4=255

#BLACK
hul5=100
huh5=154
sal5=0
sah5=60
val5=0
vah5=230

HSVLOW=np.array([hul,sal,val])
HSVHIGH=np.array([huh,sah,vah])

HSVLOW0=np.array([hul0,sal0,val0])
HSVHIGH0=np.array([huh0,sah0,vah0])

HSVLOW1=np.array([hul1,sal1,val1])
HSVHIGH1=np.array([huh1,sah1,vah1])

HSVLOW2=np.array([hul2,sal2,val2])
HSVHIGH2=np.array([huh2,sah2,vah2])  

HSVLOW3=np.array([hul3,sal3,val3])
HSVHIGH3=np.array([huh3,sah3,vah3])

HSVLOW4=np.array([hul4,sal4,val4])
HSVHIGH4=np.array([huh4,sah4,vah4])

HSVLOW5=np.array([hul5,sal5,val5])
HSVHIGH5=np.array([huh5,sah5,vah5])

#begin our 'infinite' while loop
while not rospy.is_shutdown(): 
    cv_imageBlurred = cv2.GaussianBlur(cv_image,(5,5),0)
    hsv=cv2.cvtColor(cv_imageBlurred, cv2.COLOR_BGR2HSV)
    finalSignature = [0,0,0,0,0,0,0]

    #create a mask for that range
    mask = cv2.inRange(hsv,HSVLOW, HSVHIGH)
    mask0 = cv2.inRange(hsv,HSVLOW, HSVHIGH)
    mask1 = cv2.inRange(hsv,HSVLOW1, HSVHIGH1)
    mask2 = cv2.inRange(hsv,HSVLOW2, HSVHIGH2)
    mask3 = cv2.inRange(hsv,HSVLOW3, HSVHIGH3)
    mask4 = cv2.inRange(hsv,HSVLOW4, HSVHIGH4)
    mask5 = cv2.inRange(hsv,HSVLOW5, HSVHIGH5)
    mask = mask[cropY:964,0:1288]
    mask0 = mask0[cropY:964,0:1288]
    mask1 = mask1[cropY:964,0:1288]
    mask2 = mask2[cropY:964,0:1288]
    mask3 = mask3[cropY:964,0:1288]
    mask4 = mask4[cropY:964,0:1288]
    mask5 = mask5[cropY:964,0:1288]
    cv_imageBlurred2 = cv_imageBlurred[cropY:964,0:1288]

    res = cv2.bitwise_and(cv_imageBlurred2,cv_imageBlurred2, mask = mask)
    res0 = cv2.bitwise_and(cv_imageBlurred2,cv_imageBlurred2, mask = mask0)
    res1 = cv2.bitwise_and(cv_imageBlurred2,cv_imageBlurred2, mask = mask1)
    res2 = cv2.bitwise_and(cv_imageBlurred2,cv_imageBlurred2, mask = mask2)
    res3 = cv2.bitwise_and(cv_imageBlurred2,cv_imageBlurred2, mask = mask3)
    res4 = cv2.bitwise_and(cv_imageBlurred2,cv_imageBlurred2, mask = mask4)
    res5 = cv2.bitwise_and(cv_imageBlurred2,cv_imageBlurred2, mask = mask5)
    contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    contours0, hierarchy0 = cv2.findContours(mask0, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    contours1, hierarchy1 = cv2.findContours(mask1, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    contours2, hierarchy2 = cv2.findContours(mask2, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    contours3, hierarchy3 = cv2.findContours(mask3, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    contours4, hierarchy4 = cv2.findContours(mask4, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    contours5, hierarchy5 = cv2.findContours(mask5, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

    if contours or contours2 or contours1 or contours3 or contour4 or contour5 or contour0:
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
        if rect[1][0] * rect[1][1] > minLineNoise:          
          sigX,sigY = rect[0][0] - math.cos(math.radians(rect[2]))*rect[1][1]/2, rect[0][1]+cropY + math.sin(math.radians(rect[2]))*rect[1][1]/2

          if not (sigY < lineYLeftIgnore and sigX < lineXLeftIgnore and finalSignature[4] < sigY) or not(sigX < 800 and sigY < 300):
            detect_flag = True
            finalSignature = [frameNumber,1,sigNum,rect[0][0],rect[0][1]+cropY,rect[1][1]/2,rect[2]]

      for index,contour in enumerate(contours0):
        sigNum += 1
        rect = cv2.minAreaRect(contour)
        #print rect[0][0] - rect[1][1]/2 , rect[0][1]+cropY
        box = cv2.cv.BoxPoints(rect)
        box = np.int0(box)
        for z in range(4):
          box[z][1] += cropY
        if rect[1][0] * rect[1][1] > minLineNoise:          
          sigX,sigY = rect[0][0] - math.cos(math.radians(rect[2]))*rect[1][1]/2, rect[0][1]+cropY + math.sin(math.radians(rect[2]))*rect[1][1]/2

          if not (sigY < lineYLeftIgnore and sigX < lineXLeftIgnore and finalSignature[4] < sigY) or not(sigX < 800 and sigY < 300):
            detect_flag = True
            finalSignature = [frameNumber,1,sigNum,rect[0][0],rect[0][1]+cropY,rect[1][1]/2,rect[2]]       

      for index, contour in enumerate(contours2):
        sigNum += 1
        x,y,w,h = cv2.boundingRect(contour)
        y += cropY
        if(w*h > minObstacleNoise):
          sigX,sigY = x, y + h
          detect_flag = True
          if sigX + w > obstacleXLeftIgnore:
            if ((finalSignature[1] > 1 and finalSignature[4] < sigY) or finalSignature[1] <= 1) and cutOffY < sigY:
              finalSignature = [frameNumber, 3, sigNum, sigX, sigY, w, h]

      for index,contour in enumerate(contours1):
        sigNum += 1
        x,y,w,h = cv2.boundingRect(contour)
        y += cropY
        if(w*h > minObstacleNoise):
          sigX,sigY = x, y + h
          detect_flag = True
          if sigX + w > obstacleXLeftIgnore:
            if ((finalSignature[1] > 1 and finalSignature[4] < sigY) or finalSignature[1] <= 1) and cutOffY < sigY:
              finalSignature = [frameNumber, 2, sigNum, sigX, sigY, w, h]
    
      for index, contour in enumerate(contours3):
        sigNum += 1
        x,y,w,h = cv2.boundingRect(contour)
        y += cropY
        if(w*h > minObstacleNoise):
          sigX,sigY = x, y + h
          detect_flag = True
          if sigX + w > obstacleXLeftIgnore:
            if ((finalSignature[1] > 1 and finalSignature[4] < sigY) or finalSignature[1] <= 1) and cutOffY < sigY:
              finalSignature = [frameNumber, 4, sigNum, sigX, sigY, w, h]
    
      for index, contour in enumerate(contours4):
        sigNum += 1
        x,y,w,h = cv2.boundingRect(contour)
        y += cropY
        if(w*h > minObstacleNoise):
          sigX,sigY = x, y + h
          detect_flag = True
          if sigX + w > obstacleXLeftIgnore:
            if ((finalSignature[1] > 1 and finalSignature[4] < sigY) or finalSignature[1] <= 1) and cutOffY < sigY:
              finalSignature = [frameNumber, 5, sigNum, sigX, sigY, w, h]

      for index, contour in enumerate(contours5):
        sigNum += 1
        x,y,w,h = cv2.boundingRect(contour)
        y += cropY
        if(w*h > minObstacleNoise):
          sigX,sigY = x, y + h
          detect_flag = True
          if sigX + w > obstacleXLeftIgnore:
            if ((finalSignature[1] > 1 and finalSignature[4] < sigY) or finalSignature[1] <= 1) and cutOffY < sigY:
              finalSignature = [frameNumber, 6, sigNum, sigX, sigY, w, h]

      if not detect_flag:
        finalSignature = [0,0,0,0,0,0,0]
    else:
        finalSignature = [0,0,0,0,0,0,0]

    
    sigArray.data = finalSignature
    pub.publish(sigArray)
    cv2.waitKey(1)
    rate.sleep() #if CTRL-C is pressed then exit the loop
        
 
cv2.destroyAllWindows()
