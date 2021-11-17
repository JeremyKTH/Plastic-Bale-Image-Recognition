#!/usr/bin/env python3.8
import rospy
from volvo.msg import bbox
from volvo.srv import hitta_bbox, hitta_bboxResponse
from darknet_ros_msgs.msg import BoundingBoxes

"""
# Node Name: bboxFinder
# Subscirbe: /darknet_ros/bounding_boxes
# Publish  : /bbox_info
# Task     : Retrieve bbox coordinates from darknet and filter the message
# -------------------------------------------------------------------------
# Note     : depth.py subscribes to this node
"""
xCen, yCen = 0

# Subscriber callback func
def callback(data):
    global xCen, yCen
    if data is not None:
        for i in range(len(data.bounding_boxes)):
            
            #print((data.bounding_boxes))
            
            temp_arr = str(data.bounding_boxes[i]).split("\n")

            # Get the detection type
            type_arr = temp_arr[0].split(":")
            name = type_arr[1]

            # Get the bouding box coordinates --X
            xmin_arr = temp_arr[1].split(":")
            xmin = int(xmin_arr[1])
            xmax_arr = temp_arr[3].split(":")
            xmax = int(xmax_arr[1])
            xCen = 0.5*(xmin+xmax)
            

            # Get the bouding box coordinates --Y
            ymin_arr = temp_arr[2].split(":")
            ymin = int(ymin_arr[1])
            ymax_arr = temp_arr[4].split(":")
            ymax = int(ymax_arr[1])
            yCen = 0.5*(ymin+ymax)
            
            print("detected: ", xCen, yCen)

            # Create message
            bbox_info = bbox()
            bbox_info.xCen = int(xCen)
            bbox_info.yCen = int(yCen)

            # Publish message
            bbox_pub.publish(bbox_info)

# Service callback func
def bbox_service():
    global xCen, yCen
    return hitta_bboxResponse(xCen, yCen)
    

if __name__ == '__main__':
    rospy.init_node('bboxFinder', anonymous=False)

    # Publish bbox info to depth.py
    bbox_pub = rospy.Publisher("bbox_info", bbox, queue_size=10)
    
    # Create a service for getting system states
    s = rospy.Service('hitta_bbox_service', hitta_bbox, bbox_service)
    
    # Subscribe bbox info
    rospy.Subscriber("darknet_ros/bounding_boxes", BoundingBoxes, callback)

    rospy.spin()
