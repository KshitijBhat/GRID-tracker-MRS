#!/usr/bin/env python3

## Just a publisher of coordinates.

import rospy

import numpy as np
import cv2 as cv

import pyautogui

from aruco_pose import get_pose
#from rospy.numpy_msg import numpy_msg
#from rospy_tutorials.msg import Floats
#from std_msgs.msg import Float32MultiArray
from pose_messages.msg import Pose as Poses


def TrackPose():
    pub = rospy.Publisher("track_pose",Poses, queue_size=1)
    rospy.init_node("Tracker",anonymous=True) 
    rate = rospy.Rate(10)
     
    #address = "http://192.168.1.101:4747/mjpegfeed?640x480" 
    #cam = cv.VideoCapture(address)  
        
    is_got = False
    state = [0.0,0.0,0.0]
    while not rospy.is_shutdown():

        image = pyautogui.screenshot()
        #bo,image = cam.read() 
        frame = np.array(image)
        image = cv.cvtColor(frame,cv.COLOR_BGR2RGB)

        pose,is_got = get_pose(image)
        cv.imshow('cam', cv.flip(image,1))
        if is_got:
            rospy.loginfo("Pose is being published")
            Pose = Poses()   
            Pose.x = pose[0]
            Pose.y = pose[1]
            Pose.theta = pose[2]
            pub.publish(Pose)
            print(pose)
            state = pose
        else:
            rospy.loginfo("Pose is not being published")
            Pose = Poses()   
            Pose.x = state[0]
            Pose.y = state[1]
            Pose.theta = state[2]
            pub.publish(Pose)
            print(state)

            

        rate.sleep()
            
                
            
        if cv.waitKey(1) & 0xFF == ord('x'):
            cv.destroyAllWindows()
            break 

if __name__ == "__main__":
    try:
        TrackPose()
    except rospy.ROSInterruptException:
        pass

            
    

  



    
