#!/usr/bin/env python
# license removed for brevity
import rospy, time
from std_msgs.msg import Bool
from sensor_msgs.msg import Joy

pub = rospy.Publisher('button_state', Bool, queue_size=1)
prevButtonReading = 0
pushState = False

def button_callback(data):
    global prevButtonReading
    global pushState    
    if data.buttons[1] == 1 and prevButtonReading == 0:
        #check both current & past button reading;
        #previous button reading is needed so that, if button is pushed for a long time, 
        #only 1 state change is made, regardless of the push duration
        pushState = not (pushState) 	#toggle the state variable
        prevButtonReading = 1
    elif data.buttons[1] == 0:
        prevButtonReading = 0
    pub.publish(pushState)
    
def button_talker():
    rospy.init_node('button_talker',anonymous=True)
    rospy.Subscriber('joy', Joy, button_callback)
    rospy.spin()

 
if __name__ == '__main__':
    try:
        button_talker()
    except rospy.ROSInterruptException:
        print "exiting"        

