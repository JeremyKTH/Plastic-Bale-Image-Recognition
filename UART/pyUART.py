#!/usr/bin/env python
from codecs import xmlcharrefreplace_errors
import rospy
from WheelLoaderController4 import Controller, Observer
from volvo.msg import state_message
from volvo.srv import cam_pos, cam_posResponse
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
U = UART("/dev/ttyTHS0")
if(not U.running):
    sys.exit("Could not open serial port.")
C = Controller(U)
O = Observer(U)
C.startThread()
O.startThread()


def state_service():
    global x, y
    x *= 100
    return cam_posResponse(x, y)
                       

def Callback(message):
    global C, O, x, y
    C.velocity = mapValue(float(message.velocity), 0.2199, -0.2199)
    C.steering_ang = mapValue(float(message.steering_angle), 12.903, -12.903)
    C.bucket_ang = mapValue(float(message.bucket_angle), 60, -46)
    C.bucket_height = mapValue(float(message.bucket_height), 477, -75)
    C.scissor_ang = mapValue(float(message.scissor_angle), 35, 0)
    C.Run(3)
    
    # Get states
    x, y, _, _, _, _, _, _, _, _, _, _ = O.getStates()
    
    


if __name__ == '__main__':

    rospy.init_node('pyUART', anonymous = False)
    
    # Create a service for getting system states
    s = rospy.Service('state_service', cam_pos, state_service)
    
    rospy.Subscriber('py_UART', state_message, Callback)
    
    rospy.spin()
