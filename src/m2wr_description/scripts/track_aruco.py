#!/usr/bin/env python3

import rospy

import numpy as np

import matplotlib.pyplot as plt
from rospy.numpy_msg import numpy_msg
from std_msgs.msg import Float64
traj = []

def subscriberCallBack(data):
    rospy.loginfo(rospy.get_caller_id() + " I recieved {}".format(data.data))
    traj.append(data.data)


def listener():
    rospy.init_node("subscriber", anonymous=True)
    rospy.Subscriber('track_pose',numpy_msg(Float64),subscriberCallBack)
    print(".")
    rospy.spin()
   


def main():  
    try:
        listener()
    except rospy.ROSInterruptException:
        pass    

    # plot
    [xo,yo,t0] = traj[0]
    [xf,yf,tf] = traj[-1]
    plt.figure()
    xs,ys,ts = zip(*traj)
    plt.plot(xs-xo,ys-yo,label='Tracked')
    #plt.plot(x,y,label='Reference')
    plt.quiver(xo-xo,yo-yo, np.cos(t0), np.sin(t0),scale=12,color='g')
    plt.quiver(xf-xo,yf-yo, np.cos(tf), np.sin(tf),scale=12,color='r')

    plt.title('Tracked path')
    plt.legend()
    plt.grid() 
    plt.show()

if __name__ == "__main__":
    main()    
  



    
