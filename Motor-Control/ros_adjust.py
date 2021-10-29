#!/usr/bin/env python
import rospy
import math
import numpy as np
from volvo.msg import bbox
from volvo.msg import bbox_pos
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

''' Comments regarding the code:


1: Goal is to be straight in front of the plastic bale in order to pick it up.

    1.1: There will be a limit of how centered the plastic bale needs to be.

    1.2: The bucket will have to be lowered before it drives forward.

2: Drive forward and pick up the bale.


'''


"""
# Node Name: pickup_bale
# Subscirbe: /bbox_distance
# Publish  : /bbox_dist
"""

def callback_drive(data):

    global xCen, yCen
    leftcam_to_center = 0.05

    xCen = data.xCen - 0.05    #Might be plus, not sure of the sign
    yCen = data.yCen


    if abs(xCen) <= 0.02 && yCen <= 0.5:

        print("Aligned well")

        """
        Move forward. Drive straight.
        """


        if abs(xCen) <= 0.02 && yCen <= 0.3:
            print("Stop:")
            print("Lower arm")
            print("Drive forward slowly")

            """
            After calculated distance (approx. 0.3m), the bale should be in the bucket.

            """

    else:

        """
        Has to back up a bit and then re-adjust itself in order to be aligned.

        """
        if xCen < -0.02:

            """
            Write code to back up necessary distance and adjust.
            """
            print("Too Low: center of camera is  under -0.02. : xCen= ", xCen)

        else if xCen > 0.02:

            """
            Write code to back up necessary distance and adjust.
            """

            print("Too High: center of camera is  under -0.02. : xCen= ", xCen)


def drive_func():
    # Get bbox coordinates
    rospy.Subscriber('bbox_dist', bbox, callback_drive)

    rospy.spin()


if __name__ == '__main__':
    rospy.init_node('pickup_bale', anonymous=False)

    pos_to_map = rospy.Publisher("pickup_bale", bbox_pos, queue_size=10)

    drive_func()
