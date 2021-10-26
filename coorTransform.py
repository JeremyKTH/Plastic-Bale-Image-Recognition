#!/usr/bin/env python3.8
import rospy
import tf2_ros
import tf2_msgs.msg
from geometry_msgs.msg import PoseStamped, TransformStamped
from volvo.msg import bbox_pos
    

def callback_cam_pos(data):
    # Camera state [posX. posY, posZ, quatX, quatY, quatZ, quatW]
    global cam_state
    
    cam_state = [data.pose.position.x, data.pose.position.y, data.pose.position.z, 
                 data.pose.orientation.x, data.pose.orientation.y, data.pose.orientation.z, data.pose.orientation.w]
    
    
    
    rospy.Subscriber('bbox_dist', bbox_pos , callback_bbox_pos)

    
    
def callback_bbox_pos(data): 


    if data.y != 0:


    
    # cam_state[0] += (data.y/100)
    # cam_state[1] += (data.x/100)
        x = 0
        y = 0

        x =  (data.y/100)
        y = (data.x/100)

        #print("X: ", x "Y: ", y)
        #print("Camera: ", cam_state[0], "Bale: ", (data.y/100))

        #print("Pos x: ", x ," Pos y: ", y)
        
        # Create Message
        t = TransformStamped()
        
        t.header.frame_id = "zed2_left_camera_frame"
        t.header.stamp = rospy.Time.now()
        t.child_frame_id = "bbox_frame"
        t.transform.translation.x = x
        t.transform.translation.y = -y
        t.transform.translation.z = 0

        t.transform.rotation.x = 0
        t.transform.rotation.y = 0
        t.transform.rotation.z = 0
        t.transform.rotation.w = 1

        tfm = tf2_msgs.msg.TFMessage([t])
        
        # Publish Message
        bboxCoorTF.publish(tfm)

        x = 0
        y = 0


 

def trans_func():
    rospy.Subscriber('/zed2/zed_node/pose', PoseStamped , callback_cam_pos)
   
    
    

    rospy.spin()

    
    
    

if __name__ == '__main__':
    rospy.init_node('bboxTF', anonymous=False)
    
    bboxCoorTF = rospy.Publisher("/tf", tf2_msgs.msg.TFMessage, queue_size=1)
    trans_func()
    