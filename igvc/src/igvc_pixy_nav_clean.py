#!/usr/bin/env python
# license removed for brevity
import rospy, time
from std_msgs.msg import Bool
from sensor_msgs.msg import Joy
from std_msgs.msg import Int16MultiArray
from geometry_msgs.msg import Twist

############# Global Variables #####################
go_flg = False
turn_left = False
turn_right = False
pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
moveCmd = Twist()
controller = Joy()
pixy_yAxisFrameMin = 0 # pixy y-axis pixels vary from 0 to 199 (from top to bottom)
pixy_yAxisFrameMax = 199
pixy_minAreaSize = 400
pixy_xAxisLeftThres = 0 #pixy x-axis pixels range from 0 to 319 (from left to right)
pixy_xAxisRightThres = 319
pixy_xAxisMiddle = 160
presentFrameNum = 0 
presentSignature = [0,0,0,0,0,0,0] #format of sig. list:  frame, type, signatureNum, x, y, width, height
previousSignature = [0,0,0,0,0,0,0]

############# Functions (non-callbacks) ############

def turnLeft():
    global moveCmd    
    moveCmd.linear.x = 0.25
    moveCmd.linear.y = 0
    moveCmd.linear.z = 0
    moveCmd.angular.x = 0
    moveCmd.angular.y = 0
    moveCmd.angular.z = -0.5
    pub.publish(moveCmd)

def turnRight():
    global moveCmd    
    moveCmd.linear.x = 0.25
    moveCmd.linear.y = 0
    moveCmd.linear.z = 0
    moveCmd.angular.x = 0
    moveCmd.angular.y = 0
    moveCmd.angular.z = 0.5
    pub.publish(moveCmd)

def goStraight():
    global moveCmd    
    moveCmd.linear.x = 0.5
    moveCmd.linear.y = 0
    moveCmd.linear.z = 0
    moveCmd.angular.x = 0
    moveCmd.angular.y = 0
    moveCmd.angular.z = 0
    pub.publish(moveCmd)

def reverse():
    global moveCmd    
    moveCmd.linear.x = -0.5
    moveCmd.linear.y = 0
    moveCmd.linear.z = 0
    moveCmd.angular.x = 0
    moveCmd.angular.y = 0
    moveCmd.angular.z = 0
    pub.publish(moveCmd)

def stop():
    global moveCmd    
    moveCmd.linear.x = 0
    moveCmd.linear.y = 0
    moveCmd.linear.z = 0
    moveCmd.angular.x = 0
    moveCmd.angular.y = 0
    moveCmd.angular.z = 0
    pub.publish(moveCmd)
    


############# Callbacks #############################
def go_stop_callback(button):
    global go_flg
    go_flg = button.data    


def pixy_callback(sig):
    global pixy_yAxisFrameMin
    global pixy_yAxisFrameMax
    global pixy_minAreaSize
    global presentSignature
    global presentFrameNum
    presentSignature = sig.data
    presentFrameNum = sig.data[0]
    
def button_callback(sig):
    global controller
    controller = sig
        
############# Node Definitions ######################
def pixy_nav():
    rospy.init_node('gvrbot_pixy_nav',anonymous=True)
    rospy.Subscriber('joy',Joy,button_callback)
    rospy.Subscriber('button_state', Bool, go_stop_callback)
    rospy.Subscriber('pixy_signature', Int16MultiArray, pixy_callback)
 
if __name__ == '__main__':
    try:
        pixy_nav()
        rate = rospy.Rate(145)
        loopCnt = 0
        previousSignature = [0,0,0,0,0,0,0]
        while not rospy.is_shutdown():
            loopCnt += 1
            #state machine
            if go_flg:
                print(presentSignature)
                if presentSignature[3] < 200 or (presentSignature[5] > 200):
                    print("L\n")
                    turnLeft()
                elif presentSignature[3] > 200 or (presentSignature[5] < 50):
                    print("R\n")
                    turnRight()
                else: 
                    print("S\n")
                    goStraight()
                previousSignature = presentSignature                        
            else:
                #joystick push button ("A") to go
                if len(controller.buttons) > 1 and controller.buttons[0] == 1:
                    global moveCmd    
                    moveCmd.linear.x = controller.axes[1]
                    moveCmd.linear.y = 0
                    moveCmd.linear.z = 0
                    moveCmd.angular.x = 0
                    moveCmd.angular.y = 0
                    moveCmd.angular.z = -(controller.axes[0])
                    pub.publish(moveCmd)
                else:
                    stop()
                                 

            rate.sleep()    
    except rospy.ROSInterruptException:
        print "exiting"
    finally:
        moveCmd.linear.x = 0
        moveCmd.linear.y = 0
        moveCmd.linear.z = 0
        pub.publish(moveCmd)
                
