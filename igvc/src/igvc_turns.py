<<<<<<< HEAD
#!/usr/bin/env python
# license removed for brevity
import rospy, time
from std_msgs.msg import Bool
from sensor_msgs.msg import Joy
from std_msgs.msg import Int16MultiArray
from geometry_msgs.msg import Twist

############# Global Variables #####################
go_flg = False
pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
moveCmd = Twist()
controller = Joy()

############# Functions (non-callbacks) ############

#left is negative
#right is positive
def turn(x):
    global moveCmd    
    moveCmd.linear.x = 0.5
    moveCmd.linear.y = 0
    moveCmd.linear.z = 0
    moveCmd.angular.x = 0
    moveCmd.angular.y = 0
    moveCmd.angular.z = x
    pub.publish(moveCmd)

def turnLeft():
=======
# #!/usr/bin/env python
# # license removed for brevity
# import rospy, time
# from std_msgs.msg import Bool
# from std_msgs.msg import Int16MultiArray
# from geometry_msgs.msg import Twist

# ############# Global Variables #####################
# go_flg = False
# pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
# moveCmd = Twist()

# ############# Functions (non-callbacks) ############

# #left is negative
# #right is positive
# def turn(x):
#     global moveCmd    
#     moveCmd.linear.x = 0.25
#     moveCmd.linear.y = 0
#     moveCmd.linear.z = 0
#     moveCmd.angular.x = 0
#     moveCmd.angular.y = 0
#     moveCmd.angular.z = x
#     pub.publish(moveCmd)

# def turnLeft():
#     global moveCmd    
#     moveCmd.linear.x = 0.25
#     moveCmd.linear.y = 0
#     moveCmd.linear.z = 0
#     moveCmd.angular.x = 0
#     moveCmd.angular.y = 0
#     moveCmd.angular.z = -0.5
#     pub.publish(moveCmd)

# def turnRight():
#     global moveCmd    
#     moveCmd.linear.x = 0.25
#     moveCmd.linear.y = 0
#     moveCmd.linear.z = 0
#     moveCmd.angular.x = 0
#     moveCmd.angular.y = 0
#     moveCmd.angular.z = 0.5
#     pub.publish(moveCmd)

# def goStraight():
#     global moveCmd    
#     moveCmd.linear.x = 0.5
#     moveCmd.linear.y = 0
#     moveCmd.linear.z = 0
#     moveCmd.angular.x = 0
#     moveCmd.angular.y = 0
#     moveCmd.angular.z = 0
#     pub.publish(moveCmd)

# def goBack():
#     global moveCmd    
#     moveCmd.linear.x = -0.5
#     moveCmd.linear.y = 0
#     moveCmd.linear.z = 0
#     moveCmd.angular.x = 0
#     moveCmd.angular.y = 0
#     moveCmd.angular.z = 0
#     pub.publish(moveCmd)

# def stop():
#     global moveCmd    
#     moveCmd.linear.x = 0
#     moveCmd.linear.y = 0
#     moveCmd.linear.z = 0
#     moveCmd.angular.x = 0
#     moveCmd.angular.y = 0
#     moveCmd.angular.z = 0
#     pub.publish(moveCmd)
    


# ############# Callbacks #############################
# def go_stop_callback(button):
#     global go_flg
#     go_flg = button.data   
    
        
# ############# Node Definitions ######################
# def turning():
#     rospy.init_node('gvrbot_turns',anonymous=True)
#     rospy.Subscriber('button_state', Bool, go_stop_callback)
#     rospy.Subscriber('joy',Joy,button_callback)
    
 
# if __name__ == '__main__':
#     try:
#         turning()
#         rate = rospy.Rate(50)
#         loopCnt = 0
#         while not rospy.is_shutdown():
#             left = 0
#             right = 0
#             if go_flg:

#                 #half left
#                 #if button[2] == 1 and button[0]:
#                 if len(controller.buttons)>1 and controller.buttons[2] == 1 and controller.buttons[0] == 1:
#                     turn(-0.25)
#                     print "Half Left"

#                 #half right
#                 #if button[2] == 1 and button[1]:
#                 elif len(controller.buttons)>1 and controller.buttons[2] == 1 and controller.buttons[1] == 1:
#                     turn(0.25)
#                     print "Half Right"
#                 #left
#                 #if button[0] == 1:
#                 elif len(controller.buttons)>1 and controller.buttons[0] == 1:
#                     turnLeft() 
#                     print "Left"

#                 #right  
#                 #if button[1] == 1:          
#                 elif len(controller.buttons)>1 and controller.buttons[1] == 1
#                     turnRight()
#                     print "Right"

#                 #straight
#                 #if button[2] == 1:
#                 elif len(controller.buttons)>1 and controller.buttons[2] == 1
#                     goStraight()
#                     print "Straight"

#                 #reverse
#                 #if button[3] == 1:
#                 elif len(controller.buttons)>1 and controller.buttons[3] == 1
#                     goBack() 
#                     print "Reverse"

#             else:
#                 #joystick push button ("B") stop
#                 stop()

#             rate.sleep()    
#     except rospy.ROSInterruptException:
#         print "exiting"
#     finally:
#         moveCmd.linear.x = 0
#         moveCmd.linear.y = 0
#         moveCmd.linear.z = 0
#         pub.publish(moveCmd)
                

#!/usr/bin/env python
import rospy, math, time
from std_msgs.msg import Bool
from sensor_msgs.msg import Joy, NavSatFix
from std_msgs.msg import Int16MultiArray
from geometry_msgs.msg import Twist

############## Global Variables #####################

#State/Boolean variables
state_present = 000

autonomous_flag = False
prevA = 0

gps_flag = False
prevX = 0

rightFollow_flag = True
prevY = 0

lineDetect_flag = False

objectDetect_flag = False
prevR = 0

waypoint_flag = False

destination_flag = False

gpsDecision_flag = False

#Pixy_Nav
pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
moveCmd = Twist()
controller = Joy()
presentFrameNum = 0 
presentSignature = [0,0,0,0,0,0,0] #format of sig. list:  frame, type, signatureNum, x, y, width, height
previousSignature = [0,0,0,0,0,0,0]

############# Functions (non-callback) #################

#Izzy Movement
def stop():
    global moveCmd
    moveCmd.linear.x=0
    moveCmd.linear.y=0
    moveCmd.linear.z=0
    moveCmd.angular.x=0
    moveCmd.angular.y=0
    moveCmd.angular.z=0
    pub.publish(moveCmd)
    
#variable Turn
def turn(speed, turnDirection):
    global moveCmd
    moveCmd.linear.x= speed
    moveCmd.linear.y=0
    moveCmd.linear.z=0
    moveCmd.angular.x=0
    moveCmd.angular.y=0
    moveCmd.angular.z= turnDirection
    pub.publish(moveCmd)

def turnLeft():
    #Forward and Left
>>>>>>> 2cac898a5e83ac8b35381a52e3975bdca3658231
    global moveCmd    
    moveCmd.linear.x = 0.25
    moveCmd.linear.y = 0
    moveCmd.linear.z = 0
    moveCmd.angular.x = 0
    moveCmd.angular.y = 0
    moveCmd.angular.z = -0.5
    pub.publish(moveCmd)

def turnRight():
<<<<<<< HEAD
=======
    #Forward and Right
>>>>>>> 2cac898a5e83ac8b35381a52e3975bdca3658231
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

<<<<<<< HEAD
def goBack():
=======
def reverse():
>>>>>>> 2cac898a5e83ac8b35381a52e3975bdca3658231
    global moveCmd    
    moveCmd.linear.x = -0.5
    moveCmd.linear.y = 0
    moveCmd.linear.z = 0
    moveCmd.angular.x = 0
    moveCmd.angular.y = 0
    moveCmd.angular.z = 0
    pub.publish(moveCmd)

<<<<<<< HEAD
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
    
=======
############# Callbacks #############################
#PIXY
def go_stop_callback(button):
    global go_flg
    go_flg = button.data
    global start_flg
    start_flg = button.data

def pixy_callback(sig):
    global pixy_yAxisFrameMin
    global pixy_yAxisFrameMax
    global pixy_minAreaSize
    global presentSignature
    global presentFrameNum
    presentSignature = sig.data
    presentFrameNum = sig.data[0]
>>>>>>> 2cac898a5e83ac8b35381a52e3975bdca3658231
    
def button_callback(sig):
    global controller
    controller = sig
<<<<<<< HEAD
        
############# Node Definitions ######################
def turning():
    rospy.init_node('gvrbot_turns',anonymous=True)
    rospy.Subscriber('button_state', Bool, go_stop_callback)
    rospy.Subscriber('joy',Joy,button_callback)
    
#0 = A
#1 = B
#2 = X
#3 = Y
 
if __name__ == '__main__':
    try:
        turning()
        rate = rospy.Rate(50)
        loopCnt = 0
        while not rospy.is_shutdown():
            left = 0
            right = 0
            #state machine
            if go_flg:

                #half left
                #if button[2] == 1 and button[0]:
                if len(controller.buttons)>1 and controller.buttons[2] == 1 and controller.buttons[0] == 1:
                    turn(-0.25)
                    print "Half Left"

                #half right
                #if button[2] == 1 and button[1]:
                elif len(controller.buttons)>1 and controller.buttons[2] == 1 and controller.buttons[1] == 1:
                    turn(0.25)
                    print "Half Right"
                #left
                #if button[0] == 1:
                elif len(controller.buttons)>1 and controller.buttons[0] == 1:
                    turnLeft() 
                    print "Left"

                #right  
                #if button[1] == 1:          
                elif len(controller.buttons)>1 and controller.buttons[1] == 1:
                    turnRight()
                    print "Right"

                #straight
                #if button[2] == 1:
                elif len(controller.buttons)>1 and controller.buttons[2] == 1:
                    goStraight()
                    print "Straight"

                #reverse
                #if button[3] == 1:
                elif len(controller.buttons)>1 and controller.buttons[3] == 1:
                    goBack() 
                    print "Reverse"

            else:
                #joystick push button ("B") stop
                stop()

            rate.sleep()    
    except rospy.ROSInterruptException:
        print "exiting"
    finally:
        moveCmd.linear.x = 0
        moveCmd.linear.y = 0
        moveCmd.linear.z = 0
        pub.publish(moveCmd)
                
=======

####Node Definitions####
def node_nav():
    rospy.init_node('gvrbot_pixy_nav',anonymous=True)
    rospy.Subscriber('joy',Joy,button_callback)
    rospy.Subscriber('button_state', Bool, go_stop_callback)
    rospy.Subscriber('pixy_signature', Int16MultiArray, pixy_callback)  

#################MAIN#########################
if __name__ == '__main__':
    try:
        node_nav()
        rate = rospy.Rate(140)        

        while not rospy.is_shutdown():
            ###PROCESS INPUT###

            #Autonomous Mode
            if len(controller.buttons)>1 and controller.buttons[0] == 1 and prevA == 0:
                    autonomous_flag = not (autonomous_flag)
                    prevA = 1
            elif len(controller.buttons)>1 and controller.buttons[0] ==0:
                    prevA = 0

            #Right side following ... will not be controlled manually may not be use $TODO$
            if len(controller.buttons)>1 and controller.buttons[3] == 1 and prevY == 0:
                    rightFollow_flag = not (rightFollow_flag)
                    prevY = 1
            elif len(controller.buttons)>1 and controller.buttons[3] ==0:
                    prevY = 0

            # #Obstacle Avoidance $TODO$ 
            # #RB
            # if len(controller.buttons)>1 and controller.buttons[5] == 1 and prevR == 0:
            #         objectDetect_flag = not (objectDetect_flag)
            #         prevR = 1
            # elif len(controller.buttons)>1 and controller.buttons[5] == 0:
            #         prevR = 0

            #PixyCam
            if previousSignature == presentSignature:
                lineDetect_flag = False
            else:
                lineDetect_flag = True
            presentSignature = previousSignature

            ####STATE CHART####
            # 000   ||   Manual Mode
            # 100   ||   LineFollowRight No Line
            # 105   ||   LineFollow No Line Obstacle #same code for left and right
            # 110   ||   LineFollowLeft No Line
            # 101   ||   LineFollowRight Line
            # 102   ||   LineFollowRight Line Obstacle
            # 111   ||   LineFollowLeft Line
            # 112   ||   LineFollowLeft Line Obstacle
            #
            #
                     
            ###SET STATE###
            if autonomous_flag:
                if rightFollow_flag:
                    if lineDetect_flag: 
                        if objectDetect_flag:
                            state_present = 102
                        else: 
                            state_present = 101
                    else:
                        if objectDetect_flag:
                            state_present = 105
                        else: 
                            state_present = 100
                else:
                    if lineDetect_flag: 
                        if objectDetect_flag:
                            state_present = 112
                        else: 
                            state_present = 111
                    else:
                        if objectDetect_flag:
                            state_present = 105
                        else: 
                            state_present = 110
            else:
                state_present = 000
            
            ###EXECUTE###
            #MANUAL
            if 000 == state_present:
                #Joystick push button ("A") to go
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

            #LINEFOLLOWRIGHT NO LINE
            else if 100 == state_present:
                #if obstacle: #PixyCam sees a obstacle
                    #objectDetect_flag = True
                    #if left: #the obstacle is located to the left of robot, no line
                        #turn(0.4, 0.25) #partial turn right
                        #more 
                    #else if right: #the obstacle is located to the right of robot, no line
                        #turn(0.4, -0.25) #partial turn left
                turnRight()

            #LINEFOLLOWRIGHT NO LINE OBSTACLE
            else if 105 == state_present:
                #if obstacle: #PixyCam sees a obstacle
                    #objectDetect_flag = True
                    #if left: #the obstacle is located to the left of robot, no line
                        #turn(0.4, 0.25) #partial turn right
                        #more 
                    #else if right: #the obstacle is located to the right of robot, no line
                        #turn(0.4, -0.25) #partial turn left
                if presentSignature[3] < 250 or (presentSignature[5] > 200):# or (presentSignature[3]+presentSignature[5]>250)):
                        turnLeft()
                elif presentSignature[3] > 250 or (presentSignature[5] < 50):# or (presentSignature[3]+presentSignature[5]<250)):
                        turnRight()
                else:
                    turnRight()

            #LINEFOLLOWRIGHT LINE
            else if 101 == state_present:
                if presentSignature[3] < 250 or (presentSignature[5] > 200):# or (presentSignature[3]+presentSignature[5]>250)):
                        turnLeft()
                elif presentSignature[3] > 250 or (presentSignature[5] < 50):# or (presentSignature[3]+presentSignature[5]<250)):
                        turnRight()
                else:
                    goStraight()

            #LINEFOLLOWRIGHT LINE OBSTACLE
            else if 102 == state_present:
                #if obstacle: #PixyCam sees a obstacle
                    #objectDetect_flag = True
                    #if left: #the obstacle is located to the left of robot, no line
                        #turn(0.4, 0.25) #partial turn right
                        #more 
                    #else if right: #the obstacle is located to the right of robot, no line
                        #turn(0.4, -0.25) #partial turn left
                # if presentSignature[3] < 250 or (presentSignature[5] > 200):# or (presentSignature[3]+presentSignature[5]>250)):
                #         turnLeft()
                # elif presentSignature[3] > 250 or (presentSignature[5] < 50):# or (presentSignature[3]+presentSignature[5]<250)):
                #         turnRight()
                # else:
                #     goStraight()

                if presentSignature[3] < 250 or (presentSignature[5] > 200):# or (presentSignature[3]+presentSignature[5]>250)):
                        turnLeft()
                elif presentSignature[3] > 250 or (presentSignature[5] < 50):# or (presentSignature[3]+presentSignature[5]<250)):
                        turnRight()
                else:
                    goStraight()
                    
            #LINEFOLLOWLEFT LINE OBSTACLE
            else if 112 == state_present:
                #if obstacle: #PixyCam sees a obstacle
                    #objectDetect_flag = True
                    #if left: #the obstacle is located to the left of robot, no line
                        #turn(0.4, 0.25) #partial turn right
                        #more 
                    #else if right: #the obstacle is located to the right of robot, no line
                #         #turn(0.4, -0.25) #partial turn left
                # if presentSignature[3] > 50 or (presentSignature[5] > 200):# or (presentSignature[3]+presentSignature[5]<250)):
                #         turnRight()
                # elif presentSignature[3] < 50 or (presentSignature[5] < 50):# or (presentSignature[3]+presentSignature[5]>250)):
                #         turnLeft()
                # else:
                #     goStraight()

            #LINEFOLLOWLEFT LINE
            else if 111 == state_present:
                if presentSignature[3] > 50 or (presentSignature[5] > 200):
                        turnRight()
                elif presentSignature[3] < 50 or (presentSignature[5] < 50):
                        turnLeft()
                else:
                    goStraight()

            #INVALID STATE
            else:
                stop()
            rate.sleep()
    except rospy.ROSInterruptException:
        print "exiting"
    finally:
        stop()
>>>>>>> 2cac898a5e83ac8b35381a52e3975bdca3658231
