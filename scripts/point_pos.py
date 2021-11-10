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

    global bbox_xCen, bbox_yCen, bbox_array


    bbox_xCen = data.xCen
    bbox_yCen = data.yCen

    bbox_array = data.bbox_array

    #print data.bbox_array

    # Get depth map
    rospy.Subscriber("/zed2/zed2/zed_node/depth/depth_registered", Image, callback_depth)


def callback_depth(data):

    cv_image = CvBridge().imgmsg_to_cv2(data,desired_encoding='32FC1')
    #print(cv_image.shape) # 1080x1920



    global depth, depth2, depth_array, depth_cen



    depth = cv_image[bbox_yCen, bbox_xCen]  # vertical distance to bbox
    depth2 = cv_image[0, bbox_xCen]         # vertical distance to background
    depth0 = cv_image[bbox_yCen, 0]
    depth_cen = cv_image[bbox_yCen, 960]
    depth_last = cv_image[bbox_yCen, 1920]
    depth_array = []

    for i in range(0,depth_array):

        depth_pos = cv_image[bbox_yCen, depth_array[i]]


	    depth_array.append(depth_pos)

        if depth>0.2:
	           pos_calc(depth, depth_array)




def pos_calc(depth, depth_array):


        # If bbox is at left half part of the camera's view

    if bbox_xCen < 960:
        #x = math.sqrt(3)*depth2*(960-bbox_xCen)/960
        #x /= 100        # unit: m  to cm
        #depth /= 10     # unit: mm to cm


        for i in range(1, depth_array):

            if depth_array[i]-depth_array[i-1] > 0.05:
                corner_left = depth_array[i]
                pix_left = pix_start + i


            else if (i != 0) and (depth_array[i]>depth_array[i-1]) and (depth_array[i]<depth_array[i+1]):

                corner_mid = depth_array[i]
                pix_cent = pix_start + i

            else if  depth_array[i]-depth_array[i-1] < 0.05:
                cornet_right = depth_array[i]
                pix_right = pix_start + 1



        pix_left_delta = corner_left/(np.arctan(math.pi/3))*(pix_left/1920)
        pix_cent_delta = corner_mid/(np.arctan(math.pi/3))*(pix_cent/1920)
        pix_right_delta = corner_right/(np.arctan(math.pi/3))*(pix_right/1920)


        left_ang_side = sqrt((pix_cent_delta-pix_left_delta)**2 + (corner_left-corner_mid)**2)

        right_ang_side = sqrt((pix_right_delta-pix_cent_delta)**2 + (corner_right-corner_mid)**2)

        if left_ang_side > right_ang_side:

            depth_final = depth_array[pix_start+(pix_cent_delta-pix_left_delta)/2]

            side_final = pix_left_delta + (pix_cent_delta-pix_left_delta)/2

            ang_final = np.tan(side_final/depth_final)


        else:


            depth_final = depth_array[pix_cent+(pix_right-pix_cent)/2]

            side_final = pix_cent_delta + (pix_right_delta-pix_cent_delta)/2

            ang_final = np.tan(side_final/depth_final)



    else if bbox_xCen > 960:
        #x = math.sqrt(3)*depth2*(960-bbox_xCen)/960
        #x /= 100        # unit: m  to cm
        #depth /= 10     # unit: mm to cm


        for i in range(1, depth_array):

            if depth_array[i]-depth_array[i-1] > 0.05:
                corner_left = depth_array[i]
                pix_left = pix_start + i


            else if (i != 0) and (depth_array[i]>depth_array[i-1]) and (depth_array[i]<depth_array[i+1]):

                corner_mid = depth_array[i]
                pix_cent = pix_start + i

            else if  depth_array[i]-depth_array[i-1] < 0.05:
                cornet_right = depth_array[i]
                pix_right = pix_start + 1



        pix_left_delta = corner_left/(np.arctan(math.pi/3))*(pix_left/960)
        pix_cent_delta = corner_mid/(np.arctan(math.pi/3))*(pix_cent/960)
        pix_right_delta = corner_right/(np.arctan(math.pi/3))*(pix_right/960)


        left_ang_side = sqrt((pix_cent_delta-pix_left_delta)**2 + (corner_left-corner_mid)**2)

        right_ang_side = sqrt((pix_right_delta-pix_cent_delta)**2 + (corner_right-corner_mid)**2)

        if left_ang_side > right_ang_side:

            depth_final = depth_array[pix_start+(pix_cent_delta-pix_left_delta)/2]

            side_final = pix_left_delta + (pix_cent_delta-pix_left_delta)/2

            ang_final = np.tan(side_final/depth_final)


        else:


            depth_final = depth_array[pix_cent+(pix_right-pix_cent)/2]

            side_final = pix_cent_delta + (pix_right_delta-pix_cent_delta)/2

            ang_final = np.tan(side_final/depth_final)


'Add angle to existing message type bbox_pos()'



    bbox_position = bbox_pos()
    bbox_position.x = side_final
    bbox_position.y = depth_final
    bbox_position.ang = ang_final

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
