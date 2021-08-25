# Robot Tracking using aruco

Steps:
  1. Run ```catkin build``` and ```source devel/setup.bash``` in the main folder.
  2. To launch the robot on gazebo, run ```roslaunch m2wr_description spawn.launch```
  3. To move the robot, run ```rosrun m2wr_description velocity_publisher.py``` in a new terminal. You can change the navigation commands in the code.
  4. To track its motion navigate to *aruco_detect* folder, run ```python3 screen_rec.py``` in a new tab. Minimize the Camera window. To get the final tracked path, press ```x``` while the Camera is reopened.
