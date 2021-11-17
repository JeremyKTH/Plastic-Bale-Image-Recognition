#!/usr/bin/env python3.8
import rospy
import math
import numpy as np
from volvo.msg import bbox, bbox_pos
from volvo.srv import hitta_bbox_dist, hitta_bbox_distResponse
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
position = (0, 0)

def callback_bbox(data):

    global bbox_xCen, bbox_yCen
    bbox_xCen = data.xCen
    bbox_yCen = data.yCen
    
    # Get depth map
    rospy.Subscriber("/zed2/zed_node/depth/depth_registered", Image, callback_depth)
    
 
def callback_depth(data):  
    
    cv_image = CvBridge().imgmsg_to_cv2(data,desired_encoding='32FC1')
    #print(cv_image.shape) # 1080x1920
        
    global depth, depth2
    
    depth = cv_image[bbox_yCen, bbox_xCen]  # vertical distance to bbox
    depth2 = cv_image[0, bbox_xCen]         # vertical distance to background
    
    if depth>0.1:
 
        pos_calc(depth)
    
    
    
def pos_calc(depth):
    global position
    
    # If bbox is at left half part of the camera's view
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
    
    # If bbox is straight at the front
    else:
        depth /= 10
        position = (0, depth)
        
    #print("bbox position relative to camera: ", position)
    
    
    
    # Create message
    bbox_position = bbox_pos()
    bbox_position.x = int(position[0])
    bbox_position.y = int(position[1])

    # Publish message
    pos_to_map.publish(bbox_position)
    
    
def bbox_dist_service():
    global position
    return hitta_bbox_distResponse(position[0], position[1])
    
   

if __name__ == '__main__':
    rospy.init_node('bboxDistance', anonymous=False)

    # Publish bbox distance to coorTransform.py
    pos_to_map = rospy.Publisher("bbox_dist", bbox_pos, queue_size=10)
    
    # Create a service for getting system states
    s = rospy.Service('hitta_bbox_dist_service', hitta_bbox_dist, bbox_dist_service)
    
    # Get bbox coordinates
    rospy.Subscriber('bbox_info', bbox, callback_bbox)
    
    rospy.spin()

