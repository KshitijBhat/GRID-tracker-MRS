#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
import numpy as np
import cv2 as cv
import matplotlib.pyplot as plt
import pyautogui

import trajectory
from purepursuit import PurePursuitTracker,simulate_unicycle
from aruco_pose import get_pose


#################################################################

vmax = 0.25
goal_threshold = 0.05
lookahead = 2.0

#################################################################


rospy.init_node("velocity_publisher")
pub = rospy.Publisher('/cmd_vel',Twist,queue_size = 1)

rate = rospy.Rate(10)

move = Twist()


# move.linear.x = 0.5
# move.angular.z = 0.5
x,y,t = trajectory.generate_trajectory([("straight",2),("turn",90),("straight",4),("turn",-90),("turn",-90),("turn",-90),("straight",2),("turn",-90)],radius = 1)
tracker = PurePursuitTracker(x,y,vmax) 
pose = 0, 0, np.pi/2 #arbitrary initial pose


  
traj = []      
      
while not rospy.is_shutdown():
    
    image = pyautogui.screenshot()
    frame = np.array(image)
    image = cv.cvtColor(frame,cv.COLOR_BGR2RGB)

    pose,is_got = get_pose(image)
    if is_got:
        traj.append(pose)
        print("v = ",tracker.v," w = ",tracker.w)
        move.linear.x = -tracker.v
        move.angular.z = tracker.w
        pub.publish(move)

    
        rate.sleep()
        if tracker.update(*pose):
            print("ARRIVED!!")    
            break
    #pose = simulate_unicycle(pose,tracker.v,tracker.w)    

    
move.linear.x = 0
move.angular.z = 0   
pub.publish(move) 

# plot
[xo,yo,t0] = traj[0]
[xf,yf,tf] = traj[-1]
plt.figure()
xs,ys,ts = zip(*traj)
plt.plot(xs-xo,ys-yo,label='Tracked')
plt.plot(x-xo,y-yo,label='Reference')
plt.quiver(xo-xo,yo-yo, np.cos(t0), np.sin(t0),scale=12,color='g')
plt.quiver(xf-xo,yf-yo, np.cos(tf), np.sin(tf),scale=12,color='r')

plt.title('Tracked path')
plt.legend()
plt.grid() 
plt.show()
  



    
