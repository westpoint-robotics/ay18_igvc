#!/usr/bin/env python
import rospy, math, time
from std_msgs.msg import Bool
from sensor_msgs.msg import Joy, NavSatFix
from std_msgs.msg import Int16MultiArray, Int16
from geometry_msgs.msg import Twist

#####VARIABLES TO CHANGE TO AFFECT ROBOT ACTIONS#####
#Distance from first waypoint Izzy should switch from linefollowing to gps in meters
gpsSwitchDistance = 7

#How far Izzy will see
pixyYCutOff = 100 #Higher the number means shortest view
#X and Y to make Izzy back up
pixyYBackUp = 50
pixyXBackup = 290
#X and Y to make Izzy turn Hard left
pixyYHardTurn = 80
pixyXHardLeft = 300
pixyXHardRight = 20
#X to make Izzy turn left
pixyXTurnLeft = 290
pixyXTurnRight = 5

############## Button Layout #####################
#'A'[0] - allow movement while in manual mode
#'B'[1] - toggle between autonomous and manual modes
#'Y'[3] - Manually switch between left and right side following
#'X'[2] - Change "gps coordinates" of Izzy for testing without gps component

############## Global Variables #####################

###State/Boolean variables
state_present = 000 #State code

autonomous_flag = False
prevB = 0 #Hold previous state of 'B' button on controller

gps_flag = False
prevX = 0 #Hold previous state of 'X' button on controller

rightFollow_flag = True
prevY = 0 #Hold previous state of 'Y' button on controller

lineDetect_flag = False #Is there a line present we want the robot to react to

waypoint_flag = False #Is there a waypoint for us to travel to
destination_flag = False #Have we reached the waypoint
gpsDecision_flag = False #Has A decision been made for gps line detect

###General Bot movement/control
pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
moveCmd = Twist()
controller = Joy()

###Pixy_Nav
presentFrameNum = 0
presentSignature = [0,0,0,0,0,0,0] #format of sig. list:  frame, type, signatureNum, x, y, width, height
closestSignature = [0,0]

###GPS_Nav
#Can be set manually to test changes between line following and gps states
lat = 14
lon = 14
destLat = 0
destLon = 0
robot_heading = 0
#declination angle based on geographic location
#see #https://www.ngdc.noaa.gov/geomag-web/
#needed for "grid-to-magnetic" angle
declinationAngle = 13
wayPointNum = 1     # initialize robot for getting first waypoint in the csv file

minErr = 0          #for mapping of heading error to movement commands
maxErr = 180

############# Functions (non-callback) #################

###Izzy Movement
def stop():
    global moveCmd
    moveCmd.linear.x=0
    moveCmd.linear.y=0
    moveCmd.linear.z=0
    moveCmd.angular.x=0
    moveCmd.angular.y=0
    moveCmd.angular.z=0
    pub.publish(moveCmd)

def gradRight():
    #Mostly Forward and Right
    global moveCmd
    moveCmd.linear.x = 0.25
    moveCmd.linear.y = 0
    moveCmd.linear.z = 0
    moveCmd.angular.x = 0
    moveCmd.angular.y = 0
    moveCmd.angular.z = 0.4
    pub.publish(moveCmd)

def turnLeft():
    #Forward and Left
    global moveCmd
    moveCmd.linear.x = 0.25
    moveCmd.linear.y = 0
    moveCmd.linear.z = 0
    moveCmd.angular.x = 0
    moveCmd.angular.y = 0
    moveCmd.angular.z = -0.5
    pub.publish(moveCmd)

def hardLeft():
    #Just Left
    global moveCmd
    moveCmd.linear.x = 0
    moveCmd.linear.y = 0
    moveCmd.linear.z = 0
    moveCmd.angular.x = 0
    moveCmd.angular.y = 0
    moveCmd.angular.z = -0.7
    pub.publish(moveCmd)

def turnRight():
    #Forward and Right
    global moveCmd
    moveCmd.linear.x = 0.25
    moveCmd.linear.y = 0
    moveCmd.linear.z = 0
    moveCmd.angular.x = 0
    moveCmd.angular.y = 0
    moveCmd.angular.z = 0.5
    pub.publish(moveCmd)

def hardRight():
    #Just Left
    global moveCmd
    moveCmd.linear.x = 0
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

###GPS Functions
def haversine(currentLat,currentLon, destLat, destLon):
    #Calculate the great circle distance between two points
    #on the earth (specified in decimal degrees - Lat/Lon coords) using Haversine Formula

    haversineDistance = math.acos( math.sin(currentLat*math.pi/180)*math.sin(destLat*math.pi/180) + math.cos(currentLat*math.pi/180)*math.cos(destLat*math.pi/180)*math.cos(destLon*math.pi/180-currentLon*math.pi/180) ) * 6371000

    haversineAngle = ( math.atan2(math.cos(currentLat)*math.sin(destLat)-math.sin(currentLat)*math.cos(destLat)*math.cos(destLon-currentLon), math.sin(destLon-currentLon)*math.cos(destLat)) ) * (180/math.pi)

    #transform angle perspective - Haversine calculates angle with the perspective that 90 degrees points North
    #for magnetic field reference, we need North to correspond to 0 degrees, so subtract 90
    magBearing = haversineAngle - 90
    #account for declination angle (Westerly declination, so add offset)
    magBearing = magBearing + declinationAngle
    #account for angle wrap
    if magBearing < 0:
        magBearing = magBearing + 360
    elif magBearing > 360:
        magBearing = magBearing - 360
    return haversineDistance, magBearing

def getWayPoint(wayPointNum):
    #this function iterates over each line (i.e., row) in a file containing waypoints
    #it returns the contents of the desired line (i.e., the current waypoint targeted)
    #1 input parameter:  the desired waypoint number
    #2 output parameters:  flag on whether waypoint found; and, waypoint coordinates
    count = 0               #counter keeps track of line number in file
    flg_nothingFound = True         #flag; set to true if waypoint number is NOT found
    with open("/home/user1/catkin_ws/src/igvc/src/wayPoints.csv",'r') as wayPoints: #change path to file as necessary
        for line in wayPoints:      #iterate over every line in file
            count = count + 1       #track line number of file
            if count == wayPointNum:    #check to see if this is the desired waypoint number (i.e. row)
                flg_nothingFound = False    #found a match!  Set flag.
                wayPointList = line.strip().split(',') #convert string into list of strings; slit into list based on commas in string
                wayPointList = map(float,wayPointList) #convert list of strings into list of floats
    if flg_nothingFound == True:     # didn't find line number (no more wayPoints in file)
        print "stopping robot; there are no more waypoints"
        wayPointList = [] #return empty waypoint list if waypoint number not in file
    return flg_nothingFound, wayPointList

def linearMap(val, in_min, in_max, out_min, out_max):
    return float ((val - in_min) * (out_max - out_min) / (in_max - in_min) + out_min)


def steeringController(currentHeading,desiredHeading,dist):
    # find best (i.e., shortest direction) to turn vehicle toward desired heading
    # this function compares both turning options (left & right) and finds shortest to reach desired
    # function returns "error" or how much of an angle difference between desired & actual heading
    turn_flg = True # initialize flag that robot should turn in some direction
    global moveCmd
    print "Desired: ",desiredHeading," Current: ",currentHeading
    if (desiredHeading > currentHeading):
        turnRightOption = desiredHeading - currentHeading
        turnLeftOption = currentHeading + (360 - desiredHeading)
    elif (currentHeading > desiredHeading):
        turnLeftOption = currentHeading - desiredHeading
        turnRightOption = (360 - currentHeading) + desiredHeading
    else:
        turn_flg = False # don't turn because desired = current

    #Steering Control
    if turn_flg: #if there is error, then turn. But which direction is best?
        if (turnRightOption < turnLeftOption):      #turn right
            error=turnRightOption
            moveVal = linearMap(error, minErr, maxErr, 0, .9)
            print "turn right with angular x: ", moveVal
            moveCmd.angular.z=moveVal
        else:                                       #else turn left
            error=turnLeftOption
            moveVal = linearMap(error, minErr, maxErr, 0, .9)
            moveVal = moveVal*-1    #assign as negative for publishing (-1:0 are left turn commands)
            print "turn left with angular x: ", moveVal
            moveCmd.angular.z=moveVal
    else:       #no turn necessary. Go straight
        moveCmd.angular.z=0
        print "go straight"

    #Speed Control
    if error>60: # then go slow to adjust heading
        #speed = slow;
        print "go slow...due to large angle error"
        moveCmd.linear.x=.5
    else: #check distance to determine speed
        if distance < 5: #then very close to waypoint
            #slowest
            print "slowest...close to waypoint"
            moveCmd.linear.x=.25
        elif distance < 10: #then nearing waypoint
            #go slow
            print "slow...nearing waypoint"
            moveCmd.linear.x=.5
        elif distance < 30:
            #go mod speed
            print "mod speed...between 30 to 10m"
            moveCmd.linear.x=.75
        else:
            #go fast
            print "fast speed...greater than 30m away"
            moveCmd.linear.x=1
    moveCmd.linear.y=0
    moveCmd.linear.z=0
    moveCmd.angular.x=0
    moveCmd.angular.y=0
    pub.publish(moveCmd)
    return error

############# Callbacks #############################
###PIXY
def pixy_callback(sig):
    global presentSignature
    global previousSignature
    global presentFrameNum
    global closestSignature
    final =[0,0]
    presentSignature = sig.data
    ##Checking for most immediate threat in frame
    #Check if block is in the same frame
    if presentFrameNum == presentSignature[0] and presentSignature[0] != 0:
        #Check if we care about signature
        sigList.append([presentSignature[3],presentSignature[4] + presentSignature[6])
    #If not in frame send previous frames close block
    else:
        for sign in sigList:
            if sign[1] > final[1]:
                final = sign
        closestSignature = final
        sigList = [presentSignature[3],presentSignature[4]+presentSignature[6]]
    presentFrameNum = sig.data[0]

def button_callback(sig):
    global controller
    controller = sig

###GPS
def latLon_callback(data):
    #update current gps location
    global lat
    global lon
    lat = data.latitude
    lon = data.longitude

def mag_callback(mag):
    #update current robot heading
    global robot_heading
    robot_heading=mag.data

####Node Definitions####
def node_nav():
    rospy.init_node('gvrbot_pixy_nav',anonymous=True)
    rospy.Subscriber('joy',Joy,button_callback)
    rospy.Subscriber('pixy_signature', Int16MultiArray, pixy_callback)
    rospy.Subscriber("/xsens/fix", NavSatFix, latLon_callback)
    rospy.Subscriber("tiltComp_heading",Int16, mag_callback)

#################MAIN#########################
if __name__ == '__main__':
    try:
        node_nav()
        rate = rospy.Rate(140)

        flg_nothingFound, wayPointList = getWayPoint(wayPointNum) # where wayPointlist = [lat,lon]
        if flg_nothingFound:            #if no waypoint returned, then goto "stop" state
            #print "No Starting WayPoint"
        else:
            destLat = wayPointList[0]   #extract destination coordinates from waypoint list
            destLon = wayPointList[1]
            waypoint_flag = True
        rightFollow_flag = True
        while not rospy.is_shutdown():
            ###PROCESS INPUT###
            #GPS Mode ... manual switch $$TESTING PURPOSES$$
            if len(controller.buttons)>1 and controller.buttons[2] == 1 and prevX == 0:
                    gps_flag = not (gps_flag)
                    lat -= 1
                    lon -= 1
                    prevX = 1
                    print "GPS", gps_flag
                    print "Lat", lat
                    print "Lon", lon
            elif len(controller.buttons)>1 and controller.buttons[2] ==0:
                    prevX = 0
            #Nav ... autonomous switch
            distance, desBearing = haversine(lat,lon, destLat, destLon)
            #distance = (lat + lon) - (destLat+destLon)
            if autonomous_flag and gps_flag == False and gpsSwitchDistance > distance:
                gps_flag = True
                #print "GPS ON"
            #Nav ... reach destination
            if waypoint_flag and gps_flag and distance < 1:   #reach waypoint (close enough)
                    #print "Reached Waypoint!!!"
                    destination_flag = True
                    waypoint_flag = False
            ##Autonomous Mode
            if len(controller.buttons)>1 and controller.buttons[1] == 1 and prevB == 0:
                    autonomous_flag = not (autonomous_flag)
                    prevB = 1
                    #print "Auto", autonomous_flag
            elif len(controller.buttons)>1 and controller.buttons[1] ==0:
                    prevB = 0
            #Right/Left side follow switch $$TESTING PURPOSES$$
            if len(controller.buttons)>1 and controller.buttons[3] == 1 and prevY == 0:
                    rightFollow_flag = not (rightFollow_flag)
                    prevY = 1
                    #print "Right", rightFollow_flag
            elif len(controller.buttons)>1 and controller.buttons[3] ==0:
                    prevY = 0

            ##PixyCam
            if closesetSignature = [0,0]:
                lineDetect_flag = False
                #print "No Sig"
            elif closestSignature[1] > pixyYCutOff:
                lineDetect_flag = True
                #print "Signature and true"
            else:
                lineDetect_flag = False
                #print "Signture but false"

            ####STATE CHART####
            # 000   ||   Manual Mode
            # 100   ||   LineFollowRight No Line
            # 110   ||   LineFollowLeft No Line
            # 101   ||   LineFollowRight Line
            # 111   ||   LineFollowLeft Line
            # 201   ||   GPS Get Waypoint
            # 202   ||   GPS No Line
            # 203   ||   GPS Arrived
            # 210   ||   GPS Line Decision
            # 211   ||   GPS LineFollowRight
            # 212   ||   GPS LineFollowLeft

            ###SET STATE###
            stateOut = "Manaul
            if autonomous_flag:
                if gps_flag:
                    if destination_flag:
                        state_present = 203
                        stateOut = "GPS ARRIVED"
                    elif waypoint_flag:
                        if lineDetect_flag:
                            if not gpsDecision_flag:
                                state_present = 210
                                stateOut = "GPS LINE DECISION"
                            elif rightFollow_flag:
                                state_present = 211
                                stateOut = "GPS LINEFOLLOWRIGHT"
                            else:
                                state_present = 212
                                stateOut = "GPS LINEFOLLOWLEFT"
                        else:
                            state_present = 202
                            stateOut = "GPS NOLINE"
                    else:
                        state_present = 201
                        stateOut = "GPS GET WAYPOINT"
                else:
                    if rightFollow_flag:
                        if lineDetect_flag:
                            state_present = 101
                            stateOut = "LINEFOLLOWRIGHT LINE"
                        else:
                            state_present = 100
                            stateOut = "LINEFOLLOWRIGHT NOLINE"
                    else:
                        if lineDetect_flag:
                            state_present = 111
                            stateOut = "LINEFOLLOWLEFT LINE"
                        else:
                            state_present = 110
                            stateOut = "LINEFOLLOWLEFT NOLINE"
            else:
                state_present = 000
            #print stateOut

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
            elif 100 == state_present:
                print "NOLINE"
                gradRight()
            #LINEFOLLOWLEFT NO LINE
            elif 110 == state_present:
                print "NOLINE"
                gradLeft()
            #LINEFOLLOWRIGHT LINE
            elif 101 == state_present:
                if closestSignature[1]> pixyYBackUp and closestSignature[0] < pixyXBackUp:
                    print "Back Up"
                    reverse()
                elif closestSignature[1] > pixyYHardTurn and closestSignature[0] < pixyXHardLeft:
                    print "Hard Left"
                    hardLeft()
                elif closestSignature[0] < pixyXTurnLeft:
                    print "Turn Left"
                    turnLeft()
                else:
                    goStraight()
            #LINEFOLLOWLEFT LINE
            elif 111 == state_present:
                if closestSignature[1]> pixyYBackUp:
                    print "Back Up"
                    reverse()
                elif closestSignature[1] > pixyYHardTurn and closestSignature[0] > pixyXHardRight:
                    print "Hard Right"
                    hardRight()
                elif closestSignature[0] > pixyXTurnRight:
                    print "Turn Right"
                    turnRight()
                else:
                    goStraight()
            #GPS GET WAYPOINT
            elif 201 == state_present:#get waypoint state
                nothingFound_flag, wayPointList = getWayPoint(wayPointNum)
                if nothingFound_flag:
                    stop()
                    print "No waypoint"
                print "New Waypoint"
                destLat = wayPointList[0]   #extract destination coordinates from waypoint list
                destLon = wayPointList[1]
                print destLat, destLon
                waypoint_flag = True
            #GPS NOLINE
            elif 202 == state_present:            #drive to waypoint state
                #record location data
                #get current time for logging
                timeSecs = rospy.get_time()
                with open("/home/user1/catkin_ws/posPoints.csv", "a") as posFile:
                    posFile.write(str(timeSecs)+','+str(distance)+','+str(desBearing)+','+str(robot_heading)+'\n')                           #haven't arrived; keep going
                error=steeringController(robot_heading,desBearing,distance)
                #print "heading error & distance = ",error,distance
            #GPS ARRIVED
            elif 203 == state_present:            #arrived state
                #perform arrival routine (e.g. flash LEDs); in this case, delay for 5 seconds
                #print "Stopping motors & going to sleep for 5 seconds"
                stop()
                startTime = rospy.get_rostime()
                d = rospy.Duration.from_sec(5.0)
                while (rospy.get_rostime() - startTime < d):
                    pass            #transition back to get next waypoint
                wayPointNum = wayPointNum + 1   #increment waypoint pointer to pull next target gps point
                destination_flag = False

            #GPS LINE DECISION
            elif 210 == state_present:
                if robot_heading > desBearing:
                    rightFollow_flag = False
                else:
                    rightFollow_flag = True
            #GPS LINE RIGHTFOLLOW
            elif 211 == state_present:
                if presentSignature[3] < 250 or (presentSignature[5] > 200):# or (presentSignature[3]+presentSignature[5]>250)):
                        turnLeft()
                elif presentSignature[3] > 250 or (presentSignature[5] < 50):# or (presentSignature[3]+presentSignature[5]<250)):
                        turnRight()
                else:
                    goStraight()
            #GPS LINE LEFTFOLLOW
            elif 212 == state_present:
                if presentSignature[3] > 50 or (presentSignature[5] > 200):# or (presentSignature[3]+presentSignature[5]<250)):
                        turnRight()
                elif presentSignature[3] < 50 or (presentSignature[5] < 50):# or (presentSignature[3]+presentSignature[5]>250)):
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
