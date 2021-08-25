#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
import numpy as np
import trajectory
from purepursuit import PurePursuitTracker,simulate_unicycle

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


  
      
      
while not rospy.is_shutdown():
    print("v = ",tracker.v," w = ",tracker.w)
    move.linear.x = -tracker.v
    move.angular.z = tracker.w
    pub.publish(move)

    pose = simulate_unicycle(pose,tracker.v,tracker.w)
      
    
    
    rate.sleep()
    if tracker.update(*pose):
        print("ARRIVED!!")    
        break

    
move.linear.x = 0
move.angular.z = 0   
pub.publish(move) 
