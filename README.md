# ME495 Sensing, Navigation and Machine Learning For Robotics
* Bhagyesh Agresar
* Winter 2022

# Package List

This repository consists of the following ROS Packages:

1)nuturtle_description - This package contains visualization files for turtlebot3 models and contains launch file that can load 4 turtlebot3 burger models each in different color and are loaded into rviz

2)turtlelib - turtlelib is a ROS library used to perform various 2D transformations of frames and to compute forward and inverse kinematics. The rigid2d.cpp file consists of various functions useful for calculating transformations. The diff_drive.cpp file is used to compute the forward and inverse kinematics using the functions defined in rigid2d.cpp

3)nusim - This package loads 4 red cylinders and a red turtlebot3 burger in rviz. The package is used to drive the red robot in simulation. The package publishes laser_scan data and publishes obstacles with noise.

4) nuturtle_control - This package consists of nodes such as the circle, odometry and turtle_interface. These nodes help to control the motion of the turtlebot.

5)nuslam - This package implements Extended Kalman Filter SLAM algorithm on Turtlebot3. The nuslam package consists of nuslam.cpp which is a cpp library consisting of functions required to compute the prediction and update steps of the SLAM algorithm. The slam.cpp is a slam node that executes the slam predict and update steps using the nuslam library.



# SLAM demonstration using circle
https://drive.google.com/file/d/1UBAUTxhwt_USfvPnnG-ovhaaCyD-QMpE/view?usp=sharing

# SLAM demonstration using teleop
https://drive.google.com/file/d/15R2XYYtbNbsTK84Jbqt-jAXDnkf9l9Xq/view?usp=sharing