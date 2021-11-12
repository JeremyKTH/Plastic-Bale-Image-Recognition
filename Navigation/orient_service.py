#!/usr/bin/env python3.8
import rospy
from sensor_msgs.msg import Imu
from volvo.srv import imu, imuResponse



orient_z = 0
lin_acc_y = 0

# Function to subscribe to & update the IMU information
def zed_imu_callback(data):
    # Update current orientation and linear acceleration
    now_orient_z = data.orientation.z
    now_lin_acc_y = data.linear_acceleration.y
    
    global orient_z
    global lin_acc_y
    
    # If the orientation or linear acceleration have changed within a certain tolerance, update it
    if(abs(orient_z - now_orient_z) > 0.001):
        
        orient_z = now_orient_z
        
    if(abs(lin_acc_y - now_lin_acc_y) > 0.001):
        
        lin_acc_y = now_lin_acc_y
        # lin_acc_y -= 0.12

    #imu_info = [orient_w, lin_acc_y]
    #print(imu_info)

# Function to return the current orientation and linear acceleration
def imu_service():
    
    return imuResponse(orient_z, lin_acc_y)


if __name__ == '__main__':
    # Initiate node for the service
    rospy.init_node('orient_services', anonymous=False)

    # Subscribe to IMU info, message type: Imu, calls function: zed_imu_callback
    rospy.Subscriber("/zed2/zed_node/imu/data", Imu, zed_imu_callback)
    
    # Create a service for sending the IMU info
    s = rospy.Service('imu_service', imu, imu_service)

    
    rospy.spin()