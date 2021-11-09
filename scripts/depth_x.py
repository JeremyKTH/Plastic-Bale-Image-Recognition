#!/usr/bin/env python
import rospy
import math
import numpy as np
from volvo.msg import bbox
from volvo.msg import bbox_pos
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

"""
# Node Name: bboxDistance
# Subscirbe: /bbox_info   &   /zed2/zed_node/depth/depth_registered
# Publish  : /bbox_dist
# Task     : Retrieve bbox coordinates from bbox.py and get the relative bbox distance to the camera
# --------------------------------------------------------------------------------------------------
# Note     : Should transform the coordinates to Global Frame for visualising it on rviz
"""


def callback_bbox(data):

    global bbox_xCen, bbox_yCen

    
    bbox_xCen = data.xCen
    bbox_yCen = data.yCen

    #print data.bbox_array




    
    # Get depth map
    rospy.Subscriber("/zed2/zed2/zed_node/depth/depth_registered", Image, callback_depth)
    
 
def callback_depth(data):  
    
    cv_image = CvBridge().imgmsg_to_cv2(data,desired_encoding='32FC1')
    #print(cv_image.shape) # 1080x1920
        
    global depth, depth2, depth_array

    
    
    depth = cv_image[bbox_yCen, bbox_xCen]  # vertical distance to bbox
    depth2 = cv_image[0, bbox_xCen]         # vertical distance to background
    #depth_array = []
    
    #for i in range(0,depth_array):
	#depth_array.append(int(cv_image[bbox_yCen,depth_array[i]))


    #for i in depth_array:
	


    if depth>0.2:
	pos_calc(depth)
	
    
    
    
def pos_calc(depth):
    
    
    
        # If bbox is at left half part of the camera's view

    if bbox_xCen < 960:
        #x = math.sqrt(3)*depth2*(960-bbox_xCen)/960
        #x /= 100        # unit: m  to cm
        #depth /= 10     # unit: mm to cm

	RL_frame_size = depth2*math.tan(60*math.pi/180)
	RL_pixel_size = RL_frame_size/960

	beta = math.atan((960-bbox_xCen)*RL_pixel_size/depth2)
	beta_d = beta*180/math.pi
	
	x = depth*math.tan(beta_d)

        position = (-x, depth)
        
    # If bbox is at left half part of the camera's view    
    elif bbox_xCen > 960: 
        #x = math.sqrt(3)*depth2*(bbox_xCen-960)/960
        #x = int(x/100)  # unit: m  to cm
        #depth /= 10     # unit: mm to cm
	
	RL_frame_size = depth2*math.tan(60*math.pi/180)
	RL_pixel_size = RL_frame_size/960

	beta = math.atan((bbox_xCen-960)*RL_pixel_size/depth2)
	beta_d = beta*180/math.pi
	
	x = depth*math.tan(beta_d)
	#x /= 100
	position = (x, depth)
    
    else:
        #depth /= 10
        position = (0, depth)

    print position

    '''# If bbox is at left half part of the camera's view

    if bbox_xCen < 960:
        x = math.sqrt(3)*depth2*(960-bbox_xCen)/960
        x /= 100        # unit: m  to cm
        depth /= 10     # unit: mm to cm
        position = (-x, depth)
        
    # If bbox is at left half part of the camera's view    
    elif bbox_xCen > 960: 
        x = math.sqrt(3)*depth2*(bbox_xCen-960)/960
        x = int(x/100)  # unit: m  to cm
        depth /= 10     # unit: mm to cm
        position = (x, depth)
    
    else:
        depth /= 10
        position = (0, depth)
        
    print position'''
    
    
    


    # Create message
    bbox_position = bbox_pos()
    bbox_position.x = position[0]
    bbox_position.y = position[1]

    # Publish message
    pos_to_map.publish(bbox_position)
    
    
def dis_func():
    # Get bbox coordinates
    rospy.Subscriber('bbox_info', bbox, callback_bbox)
    
    
    
    rospy.spin()
   

if __name__ == '__main__':
    rospy.init_node('bboxDistance', anonymous=False)

    pos_to_map = rospy.Publisher("bbox_dist", bbox_pos, queue_size=10)
    
    dis_func()

