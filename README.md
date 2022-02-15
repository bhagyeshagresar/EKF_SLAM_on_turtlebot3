# ME495 Sensing, Navigation and Machine Learning For Robotics
* Bhagyesh Agresar
* Winter 2022

# Package List

This repository consists of the following ROS Packages:

1)nuturtle_description - This package contains visualization files for turtlebot3 models and contains launch file that can load 4 turtlebot3 burger models each in different color and are loaded into rviz

2)turtlelib - turtlelib is a ROS library used to perform various 2D transformations of frames and to compute forward and inverse kinematics. The rigid2d.cpp file consists of various functions useful for calculating transformations. The diff_drive.cpp file is used to compute the forward and inverse kinematics using the functions defined in rigid2d.cpp

3)nusim - This package loads 4 red cylinders and a red turtlebot3 burger in rviz. The package is used to drive the red robot in simulation

4) nuturtle_control - This package consists of nodes such as the circle, odometry and turtle_interface. These nodes help to control the motion of the turtlebot.




# Kinematics Calculations

https://drive.google.com/file/d/1rcvyuzr0tUmVKinjFVewD1EDhmv9mqnn/view?usp=sharing


# Task F:

1) Experiment 1 

Real Robot:
https://drive.google.com/file/d/1rdc6YX6hns3kTKHWuZXZW5nEKnNmUwW5/view?usp=sharing


Rviz:
https://drive.google.com/file/d/1puqnFdtuyB8gV4N4JJc4Ea-b7XNQ4Lt5/view?usp=sharing


Final Pose:
https://drive.google.com/file/d/1ZHAZAq3qEu58O7alM4j4i5MhdqHugRJ2/view?usp=sharing



2) Experiment 2

Real Robot:
https://drive.google.com/file/d/1rdeTydPv2JKU-axHq0_-YLUEDfB6fWXD/view?usp=sharing


Rviz:
https://drive.google.com/file/d/1t3GbQHsmEx5mlAS-8j096-h1GuElWBpX/view?usp=sharing


Final Pose:
https://drive.google.com/file/d/1OGC24xq3axGb26Q2jAD1GEq0z8-RnGwH/view?usp=sharing


3) Experiment 3:

Robot wheels spin for an instant and stop. This happens randomly and my robot does not move in a circle continuously.



4) Bad result

I tried moving the robot in a forward direction and then suddenly rotate it clockwise and anticlockwise direction without stopping the robot. At the end the odometry resulted in the pose in the y direction at the start position to have y = -0.19m which was off as compared to the x and y position values. Another similar experiment gave z = 0.25m value. Attached results are for y = -0.19m result.

Real Robot:

https://drive.google.com/file/d/1rnM4GsfIa8eZ5vM1jEXWq_3RUb92Fla9/view?usp=sharing


Rviz:

https://drive.google.com/file/d/1fMq2IWVQHByV-7NYU3lSVQk-0-CG4qjz/view?usp=sharing


Final Pose:

https://drive.google.com/file/d/1kXjPNNiSrd4kJJl0MF6jLAU-UF8fH_yz/view?usp=sharing