#!/usr/bin/env python
import rospy
from volvo.msg import state_message
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
#state_message= [ 0, 0 ,0 ,0 ,20]

if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node('pySM', anonymous = False)
    
    # Go to the main loop.
    pub_UART = rospy.Publisher('pub_UART', state_message, queue_size=10)
    rospy.Rate(10)
    
    # Create message
    state_msg = state_message()
    state_msg.velocity = 0
    state_msg.steering_angle = 0
    state_msg.bucket_angle = 0
    state_msg.bucket_height = 0
    state_msg.scissor_angle = 20
    
    
    
    while not rospy.is_shutdown():
        # Publish message
        pub_UART.publish(state_msg)
        rospy.Rate(10).sleep()
   
