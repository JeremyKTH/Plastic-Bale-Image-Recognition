#!/usr/bin/env python3.8

from nav_functions import *
from volvo.msg import cam_pos, state_message
from WheelLoaderController4 import Controller, Observer

# -----------------------------------------------------------
# Navigation 1.1: Drive forward

def nav_1_1(vel, forward_dist_1_1, current_pos, orient_origin, orient_threshold):

    while (current_pos < forward_dist_1_1):
        # Get orientation
        orient_curr = get_imu_info()
        
        # Check if adjustment is required
        adjust_orient(vel, orient_curr, orient_origin, orient_threshold)
    
        # Calulate distance
        current_pos, _ = get_state_info()

        # print(current_pos)
    
# -----------------------------------------------------------
# Navigation 1.2: Turn left
    
def nav_1_2(vel, steer_angle, orient_threshold):
    
    orient_curr = get_imu_info()

    while (orient_curr < (90-orient_threshold)):
        # Get orientation
        orient_curr = get_imu_info()
        #print(f'orientation difference: {90-orient_curr}, Need to turn left!')
        # Publish command to UART
        send_command(vel, steer_angle, 0, 0, 0)
        
# -----------------------------------------------------------
# Navigation 1.3: Drive forward until bale is seen
    
def nav_1_3(vel, forward_dist_1_3, current_pos, orient_origin, orient_threshold):
     
    while (current_pos < forward_dist_1_3):
        # Get orientation
        orient_curr = get_imu_info()
        
        # Check if adjustment is required
        adjust_orient(vel, orient_curr, orient_origin, orient_threshold) 
    
        # Calulate distance
        current_pos, _ = get_state_info()

# -----------------------------------------------------------    
# Navigation 1.4: Align to bale until approx. 25 cm away

def nav_1_4():
    pass
    
# -----------------------------------------------------------    
# Navigation 2: When close enough (25 cm), run Bale_PickUp_bucket.csv

def nav_2(seq_file):
    sequence_exec(seq_file)
    
# -----------------------------------------------------------    
# Navigation 3.1: Reverse with max turning radius
def nav_3_1(vel, steer_angle, orient_threshold):
    orient_curr = get_imu_info()
    
    while (80 < orient_curr < (177-orient_threshold)):
        # Get orientation
        orient_curr = get_imu_info()
        #print(f'orientation difference: {177-orient_curr}, Need to reverse right!')
        
        # Publish command to UART (reverse right)   
        send_command(-vel, -steer_angle, 0, 0, 0)
         
        
# -----------------------------------------------------------    
# Navigation 3.2: Drive forward
def nav_3_2(vel, current_pos, forward_dist_3_2, orient_threshold):
    
    while (current_pos < forward_dist_3_2):
        # Get orientation
        orient_curr = get_imu_info()
        
        # Check if adjustment is required
        adjust_orient_special(vel, orient_curr, orient_threshold) 
    
        # Calulate distance
        current_pos, _ = get_state_info()
        
# -----------------------------------------------------------    
# Navigation 4: Run sequence for dropping off bale

def nav_4(seq_file):
    sequence_exec(seq_file)    
    
# -----------------------------------------------------------    
# Navigation 5.1: Reverse right

def nav_5_1(vel, steer_angle, orient_threshold):
    orient_curr = get_imu_info()
    
    while (((177-orient_threshold) < orient_curr) or (orient_curr < (-177+orient_threshold))):
        # Get orientation
        orient_curr = get_imu_info()
        #print(f'orientation difference: {(-177+orient_threshold)-orient_curr}, Need to reverse right!')
        
        # Publish command to UART (reverse right) 
        send_command(-vel, -steer_angle, 0, 0, 0)
    

# -----------------------------------------------------------    
# Navigation 5_2: Drive forward

def nav_5_2(vel, current_pos, forward_dist_5_2, orient_origin, orient_threshold):
    while (current_pos < forward_dist_5_2):
        # Get orientation
        orient_curr = get_imu_info()
        
        # Check if adjustment is required
        adjust_orient(vel, orient_curr, orient_origin, orient_threshold)
    
        # Calulate distance
        current_pos, _ = get_state_info()    
        
# -----------------------------------------------------------    
# Navigation 5_3: Zeuxy dance

def nav_5_3():
    pass


if __name__ == '__main__':
    
    rospy.init_node('nav_sequences', anonymous = False)
    rospy.spin()
