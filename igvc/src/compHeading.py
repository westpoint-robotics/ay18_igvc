#!/usr/bin/env python

#node pulls accelerometer and magnetometer data from xsens sensor, uses data to calculate current robot magnetic heading
#publishes current robot heading to topic 'tiltComp_heading'

import rospy,time,numpy,math,subprocess
from std_msgs.msg import Int16, Header
from sensor_msgs.msg import Imu, MagneticField
from geometry_msgs.msg import Vector3, Vector3Stamped

#################Global variables#################

accX=0
accY=0
accZ=0
magX=0
magY=0
magZ=0

#################Functions Non-Callback#################
        
def calcRollAngle(accY,accZ):
    rollAngle = math.atan2(accY,accZ)
    return rollAngle

def calcPitchAngle(accX,accY,accZ,rollAngle):
    pitchAngle = math.atan2(-accX, ((accY * math.sin(rollAngle)) + (accZ * math.cos(rollAngle))))
    return pitchAngle

def tiltCompHeading(magX,magY,magZ,pitchAngle,rollAngle):
    Mx2 = (magX * math.cos(pitchAngle)) + (magZ*math.sin(pitchAngle))
    My2 = (magX * math.sin(rollAngle) * math.sin(pitchAngle)) + magY*math.cos(rollAngle) - (magZ*math.sin(rollAngle)*math.cos(pitchAngle))       
    compHeadingDeg = math.atan2(My2,Mx2) * (180/math.pi)
    if compHeadingDeg <0:       #correct for degree wrap
        compHeadingDeg = compHeadingDeg + 360
    return compHeadingDeg

#################Callback Functions#################

def imuCallback(data):
    #update accelerometer data
    global accX
    global accY
    global accZ
    accX=data.linear_acceleration.x
    accY=data.linear_acceleration.y
    accZ=data.linear_acceleration.z

def magCallback(data):
    #update magnetometer data
    global magX
    global magY
    global magZ
    magX=data.vector.x
    magY=data.vector.y
    magZ=data.vector.z

#################Subscriber/Publisher#################
    
def imuTalker():
    #initialize node, establish publishing, subscribe to necessary xsens data
    pub=rospy.Publisher("tiltComp_heading", Int16, queue_size=1)
    rospy.init_node("imu_driver", anonymous=True)
    rospy.Subscriber("/xsens/imu/data",Imu,imuCallback)
    rospy.Subscriber("/xsens/magnetic",Vector3Stamped,magCallback)
    
    rate = rospy.Rate(15) # 15hz
    compHeadingDeg = Int16()
    while not rospy.is_shutdown():
        #Calculate roll/pitch angles from accelerometer
        rollAngle = calcRollAngle(accY,accZ)
        pitchAngle = calcPitchAngle(accX,accY,accZ,rollAngle)
        #find tilt compensated heading
        compHeadingDeg = tiltCompHeading(magX,magY,magZ,pitchAngle,rollAngle)         
        #publish the data        
        pub.publish(compHeadingDeg)        
        rate.sleep()

if __name__ == '__main__':
    try:
        imuTalker()
    except rospy.ROSInterruptException:
        pass

