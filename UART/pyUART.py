#!/usr/bin/env python
import rospy
from WheelLoaderController3 import Controller, Observer
from volvo.msg import state_message
from UART import UART
from SequenceExecuter import mapValue
import sys

"""
# Node Name: pyUART
# Subscirbe: state machine   
# Publish  : stm32
# Task     : 
# --------------------------------------------------------------------------------------------------
# Note     : 
"""
U = UART("COM4")
if(not U.running):
    sys.exit("Could not open serial port.")
C = Controller(U)
O = Observer(U)
C.startThread()
O.startThread()


def Callback(message):
    global C, O
    C.velocity = mapValue(float(message.velocity), 0.2199, -0.2199)
    C.steering_ang = mapValue(float(message.steering_angle), 12.903, -12.903)
    C.bucket_ang = mapValue(float(message.bucket_angle), 60, -46)
    C.bucket_height = mapValue(float(message.bucket_height), 477, -75)
    C.scissor_ang = mapValue(float(message.scissor_angle), 35, 0)
    C.Run(3)
    #print(message)


if __name__ == '__main__':
    # Initialize the node and name it.
    rospy.init_node('pyUART', anonymous = False)

    # Go to the main loop.
    rospy.Subscriber('pub_UART', state_message, Callback)
    rospy.spin()
