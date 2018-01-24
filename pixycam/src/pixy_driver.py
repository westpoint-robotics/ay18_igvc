#!/usr/bin/env python

import rospy
from std_msgs.msg import Int16MultiArray

#for pixycam
from pixy import *
from ctypes import *

# Initialize Pixy Interpreter thread #
pixy_init()

class Blocks (Structure):
  _fields_ = [ ("type", c_uint),
               ("signature", c_uint),
               ("x", c_uint),
               ("y", c_uint),
               ("width", c_uint),
               ("height", c_uint),
               ("angle", c_uint) ]



def pixy_driver():
    pub = rospy.Publisher('pixy_signature', Int16MultiArray, queue_size=10)
    rospy.init_node('pixy_driver', anonymous=True)
    rate = rospy.Rate(100) # 30hz
    #define array for storing signature data
    sigArray = Int16MultiArray()
    #for pixy
    blocks = BlockArray(100)
    frame  = 0
    while not rospy.is_shutdown():
        count = pixy_get_blocks(100, blocks)
        if count > 0:
            # Blocks found #
            frame = frame + 1
            for index in range (0, count):
                #print '[BLOCK_TYPE=%d SIG=%d X=%3d Y=%3d WIDTH=%3d HEIGHT=%3d]' % (blocks[index].type, blocks[index].signature, blocks[index].x, blocks[index].y, blocks[index].width, blocks[index].height)
                blockInfo = [frame, blocks[index].type, blocks[index].signature, blocks[index].x, blocks[index].y, blocks[index].width, blocks[index].height]
                sigArray.data = blockInfo
                pub.publish(sigArray)
		else:
			sigArray.data = [0,0,0,0,0,0,0]
			pub.publish(sigArray)
        rate.sleep()

if __name__ == '__main__':
    try:
        pixy_driver()
    except rospy.ROSInterruptException:
        pass
