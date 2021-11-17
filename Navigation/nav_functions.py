#!/usr/bin/env python
import rospy
import numpy as np
from sensor_msgs.msg import Imu
from volvo.msg import state_message
from volvo.srv import imu, cam_pos
from time import sleep


# ---------------------------------------------------------------
# Service: Get orientation from IMU and convert to angle 

def get_imu_info():
    rospy.wait_for_service('imu_service')
    
    try:
        imu_srv = rospy.ServiceProxy('imu_service', imu)
        imu_info = imu_srv(1.0) # send in random value of bla cuz not using it
        
        print("orient: ", imu_info.orient_z)
              
        # Remap orientation from -1:1 to -177:177 degrees
        rad = 2*np.arcsin(imu_info.orient_z)
        angle = np.rad2deg(rad)
        print("rad: ", rad)
        print("angle: ", angle)
        #print("lin_acc: ", imu_info.lin_acc_y)
        
        return angle

    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)
        
# --------------------------------------------------------------- 
# Service: Get state information
 
def get_state_info():
    rospy.wait_for_service('state_service')
    
    try:
        state_srv = rospy.ServiceProxy('state_service', cam_pos)
        state_info = state_srv(1.0) # send in random value of bla cuz not using it
        
        x = state_info.x
        y = state_info.y
        
        return x, y

    except rospy.ServiceException as e:
        print("Service call failed: %s"%e)

# ---------------------------------------------------------------     
# Function: Adjust orientation

def adjust_orient(vel, orient_curr, orient_origin, orient_threshold):
    
    K_err = 1
    
    if(orient_curr > orient_origin + orient_threshold):
        orient_diff = orient_curr - (orient_origin + orient_threshold)
        orient_diff = mapValue(orient_diff)
        
        # Need to adjust right
        print("Adjust right")
        
        # Publish command to UART
        send_command(vel, K_err*orient_diff, 0, 0, 0)
          
    elif(orient_curr < orient_origin - orient_threshold):
        orient_diff = orient_curr - (orient_origin - orient_threshold)
        orient_diff = mapValue(orient_diff)
    
        # Need to adjust left   
        print("Adjust left")
        # Publish command to UART
        send_command(vel, K_err*orient_diff, 0, 0, 0)
        
    else:
        # Deviation is admissable
        print("Orientation admissable")
        # Publish command to UART
        send_command(vel, 0, 0, 0, 0)

# ---------------------------------------------------------------     
# Function: Adjust orientation special case for Navigation 3.2       
def adjust_orient_special(vel, orient_z, orient_threshold):
    K_err = 1
    
    if(0 >= orient_z >= (-177+orient_threshold)):
        
        orient_diff = orient_z - (-177+orient_threshold)
        orient_diff = mapValue(orient_diff)
        
        # Need to adjust right
        print("Adjust right")
        # Publish command to UART
        send_command(vel, K_err*orient_diff, 0, 0, 0)
        
    elif(0 <= orient_z <= (177-orient_threshold)):
        
        orient_diff = orient_z - (-177+orient_threshold)
        orient_diff = mapValue(orient_diff)
        
        # Need to adjust left   
        print("Adjust left")
        # Publish command to UART
        send_command(vel, K_err*orient_diff, 0, 0, 0)
        
    else:
        # Deviation is admissable
        print("Orientation admissable")
        # Publish command to UART
        send_command(vel, 0, 0, 0, 0)     
        
# --------------------------------------------------------------- 
# Function: Publish command to UART via state message
def send_command(vel, steer_angle, buck_angle, buck_height, sciss_angle):

    state_msg = state_message()
    state_msg.velocity = vel
    state_msg.steering_angle = steer_angle
    state_msg.bucket_angle = buck_angle
    state_msg.bucket_height = buck_height
    state_msg.scissor_angle = sciss_angle
    
    # Publish
    pub_UART = rospy.Publisher('py_UART', state_message, queue_size=10)
    rospy.Rate(10)
    
    pub_UART.pub(state_msg)

# --------------------------------------------------------------- 
# Function: Publish command to UART via state message for pickup/dropoff sequences
def send_command_seq(instr):

    state_msg = state_message()
    state_msg.velocity = instr[0]
    state_msg.steering_angle = instr[1]
    state_msg.bucket_angle = instr[2]
    state_msg.bucket_height = instr[3]
    state_msg.scissor_angle = instr[4]
    
    # Publish
    pub_UART = rospy.Publisher('pub_UART', state_message, queue_size=10)
    rospy.Rate(10)
    
    pub_UART.pub(state_msg)
    
    
# --------------------------------------------------------------- 
# Function: Read in sequence files and execute   
def sequence_exec(seq_file):
    instruction = seq_file.readline().split(',')

    while len(instruction) > 1:
        try:
            send_command_seq([float(ins) for ins in instruction])
        except (ValueError):
            instruction = seq_file.readline().split(',')
            continue
        sleep(float(instruction[5]))
        instruction = seq_file.readline().split(',')

# --------------------------------------------------------------- 
# Function to map steering angle if it's bigger/smaller than max/min angle        
def mapValue(val):
    max = 12.903
    min = -12.903
    if val > max:
        return max
    elif val < min:
        return min
    else:
        return val
        

if __name__ == '__main__':
    
    rospy.init_node('nav_functions', anonymous = False)
    
    rospy.spin()
