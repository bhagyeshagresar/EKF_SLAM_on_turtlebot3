# nuslam package:
The nuslam package contains the nuslam library for computing the predict and update steps for the Extended Kalman Filter SLAM. The package also contains slam.cpp node for implementing the EKF SLAM on turtlebot3. The slam.cpp node publishes markers based on the measurement update. The slam node subscribes to /fake_sensor to get the position of the markers
