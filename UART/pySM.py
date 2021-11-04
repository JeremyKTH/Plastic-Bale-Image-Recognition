#!/usr/bin/env python
import rospy
#import path planner ect. 


"""
# Node Name: state machine
# Subscirbe: 
# Publish  : ptUART
# Task     : 
# --------------------------------------------------------------------------------------------------
# Note     : 
"""
""" C.velocity = mapValue(float(message[0]), 0.2199, -0.2199)
    C.steering_ang = mapValue(float(message[1]), 12.903, -12.903)
    C.bucket_ang = mapValue(float(message[2]), 60, -46)
    C.bucket_height = mapValue(float(message[3]), 477, -75)
    C.scissor_ang = mapValue(float(message[4]), 35, 0)"""
state_message= [ 0, 0 ,0 ,0 ,20]

if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node('pySM', anonymous = False)

    # Go to the main loop.
    rospy.Publisher('pyUART', state_message, queue_size=10)
    rospy.Rate(10).sleep
    rospy.spin()