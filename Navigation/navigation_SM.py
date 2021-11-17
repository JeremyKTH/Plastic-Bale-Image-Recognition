#!/usr/bin/env python3.8
import rospy
from sensor_msgs.msg import Imu
from volvo.msg import state_message
from volvo.srv import imu
from time import sleep
from nav_functions import *
from nav_sequences import *



''' Comments regarding the code:

1: Navigate until in close proximity to bale

    1.1 Drive forward
    
    1.2 Turn left
    
    1.3 Drive forward until bale is seen
    
    1.4 Align to bale until approx. 30 cm awayorient_diff = mapValue(orient_diff)

2. When close enough (30 cm), run Bale_PickUp_bucket.csv

3. Navigate to drop-off point

    3.1 Reverse with max turning radius
    
    3.2 Drive forward
    
4. Run a code for dropping off bale 

5. Navigate to starting point

    5.1 Reverse with max turning radius
    
    5.2 Drive forward
    
    5.3 Stop
    
6. Zeuxy dance!

'''
    
"""
# Node Name: navigation
# Subscribe: /zed2/zed_node/imu/data
# Publish  : /pyUART
# Task     : Send command to STM32 via pyUART
"""

orient_z = 0
bbox_flag = 0
# --------------------------------------------------------------- 
# Navigation

def navigation():
    
    vel = 0.05 # m/s
    steer_angle_max = 12.903
    global bbox_flag
    
    # -----------------------------------------------------------
    # Navigation 1.1: Drive forward
    
    print("-----------------")
    print("Start section 1.1")
    print("-----------------")    
    
    forward_dist_1_1 = 2
    current_pos = 0
    orient_origin = 0
    orient_threshold = 5
    
    nav_1_1(vel, forward_dist_1_1, current_pos, orient_origin, orient_threshold)
    
    print("---------------------------")
    print("Destination 1.1 has reached")
    print("---------------------------")
    
    rospy.sleep(4.0)
    
    # -----------------------------------------------------------
    # Navigation 1.2: Turn left
    
    print("-----------------")
    print("Start section 1.2")
    print("-----------------")    
    
    nav_1_2(vel, steer_angle_max, orient_threshold)
    
    print("---------------------------")
    print("Destination 1.2 has reached")
    print("---------------------------")
        
    rospy.sleep(4.0)
    
    # -----------------------------------------------------------
    # Navigation 1.3: Drive forward until bale is seen
    
    print("-----------------")
    print("Start section 1.3")
    print("-----------------")    

    orient_origin = 90
    
    # UNTIL BALE IS SEEN
    
    nav_1_3(vel, orient_origin, orient_threshold)
        
    
    print("---------------------------")
    print("Destination 1.3 has reached")
    print("---------------------------")
    rospy.sleep(4.0)
    
    # -----------------------------------------------------------    
    # Navigation 1.4: Align to bale until approx. 25 cm away
    print("-----------------")
    print("Start section 1.4")
    print("-----------------")  
    
    nav_1_4(vel, orient_origin, orient_threshold)
    
    print("---------------------------")
    print("Destination 1.4 has reached")
    print("---------------------------")
    rospy.sleep(4.0)
    
    # -----------------------------------------------------------
    # Navigation 2: When close enough (25 cm), run Bale_PickUp_bucket.csv
    print("---------------")
    print("Start section 2")
    print("---------------")

    # Run pickup sequence
    
    print("-----------------------")
    print("Running pickup sequence")
    print("-----------------------")
    
    seq_file = open('Bale_PickUp_Sequence.csv', 'r')
    nav_2(seq_file)

    print("-----------------------")
    print("Bale has been picked up")
    print("-----------------------")
    
    rospy.sleep(4.0)   
    # -----------------------------------------------------------
    # Navigation 3.1: Reverse with max turning radius
    # Reverse rightdef mapValue(val, max, min):

    print("-----------------")
    print("Start section 3.1")
    print("-----------------")
    
    nav_3_1(vel, steer_angle_max, orient_threshold)
        
    print("---------------------------")
    print("Destination 3.1 has reached")
    print("---------------------------")
    
    rospy.sleep(4.0)    
    
    # -----------------------------------------------------------
    # Navigation 3.2: Drive forward
    print("-----------------")
    print("Start section 3.2")
    print("-----------------")
    forward_dist_3_2 = 12
    current_pos = 0

    nav_3_2(vel, current_pos, forward_dist_3_2, orient_threshold)
    
    print("---------------------------")
    print("Destination 3.2 has reached")
    print("---------------------------")
    rospy.sleep(4.0)    
    
    # -----------------------------------------------------------
    # Navigation 4: Run sequence for dropping off bale   
    
    print("---------------")
    print("Start section 4")
    print("---------------")
    
    # Run dropoff code
    
    print("------------------------")
    print("Running dropoff sequence")
    print("------------------------")    
    
    seq_file = open('Bale_DropOff_Sequence.csv', 'r')
    nav_4(seq_file)
    
    print("-------------------------")
    print("Bale has been dropped off")
    print("-------------------------")
    rospy.sleep(4.0)
    # -----------------------------------------------------------
    # # Navigation 5.1: Reverse right
    print("-----------------")
    print("Start section 5.1")
    print("-----------------")
    
    nav_5_1(vel, steer_angle_max, orient_threshold)
          
    print("---------------------------")
    print("Destination 5.1 has reached")
    print("---------------------------")
    rospy.sleep(4.0)  
      
    # -----------------------------------------------------------
    # Navigation 5.2: Drive forward
    print("-----------------")
    print("Start section 5.2")
    print("-----------------")
    
    forward_dist_5_2 = 5
    current_pos = 0
    orient_origin = -90

    nav_5_2(vel, current_pos, forward_dist_5_2, orient_origin, orient_threshold)
    
    print("---------------------------")
    print("Destination 5.2 has reached")
    print("---------------------------")
    rospy.sleep(4.0)   
    
    # -----------------------------------------------------------    
    # # Navigation 5.3: Zeuxy dance
    
    # WRITE ZEUX DANCE
    nav_5_3()
    print("-------------------")
    print("FUCK THIS, I'M DONE")
    print("-------------------") 
    
if __name__ == '__main__':
   
    rospy.init_node('navigation', anonymous=False)
    
    navigation()
    
    rospy.spin()
