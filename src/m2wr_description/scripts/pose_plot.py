#!/usr/bin/env python3

## Subscriber for pose data and plotter

import rospy

import numpy as np

import matplotlib.pyplot as plt

from pose_messages.msg import Pose

traj = []

def subscriberCallBack(Pose):

    pose = [Pose.x,Pose.y,Pose.theta]
    rospy.loginfo(rospy.get_caller_id() + " I recieved {}".format(pose))
    
    traj.append(pose)


def listener():
    rospy.init_node("subscriber", anonymous=True)
    rospy.Subscriber('track_pose',Pose,subscriberCallBack)
    rospy.spin()
   


def main():  
    try:
        listener()
    except rospy.ROSInterruptException:
        pass    

    # plot
    print(traj)
    [xo,yo,t0] = traj[0]
    [xf,yf,tf] = traj[-1]
    plt.figure()
    xs,ys,ts = zip(*traj)
    xns = np.array(xs) - xo
    yns = np.array(ys) - yo
    plt.plot(xns,yns,label='Tracked')
    #plt.plot(x,y,label='Reference')
    plt.quiver(0,0, np.cos(t0), np.sin(t0),scale=12,color='g')
    xf-=xo
    yf-=yo
    plt.quiver(xf,yf, np.cos(tf), np.sin(tf),scale=12,color='r')

    plt.title('Tracked path')
    plt.legend()
    plt.grid() 
    plt.show()

if __name__ == "__main__":
    main()    
  



    
