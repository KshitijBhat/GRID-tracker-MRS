# Robot Tracking using aruco

Steps:
  1. Create a catkin workspace and add *m2wr_discription* in the *src* folder
  2. Run ```catkin build``` and ```source devel/setup.bash```
  3. To launch the robot on gazebo, run ```roslaunch m2wr_description spawn.launch```
  4. To move the robot, run ```rosrun m2wr_description velocity_publisher.py``` in a new terminal. You can change the navigation commands in the code.
  5. To track its motion navigate to *aruco_detect* folder, run ```python3 screen_rec.py``` in a new tab. Minimize the Camera window. To get the final tracked path, press ```x``` while the Camera is reopened.
