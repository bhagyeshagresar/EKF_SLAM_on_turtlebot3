// #include <ros/ros.h>
// #include <geometry_msgs/Twist.h>
// #include "nuturtlebot_msgs/WheelCommands.h"
// #include "nuturtlebot_msgs/SensorData.h"
// #include <sensor_msgs/JointState.h>
// #include "turtlelib/diff_drive.hpp"
// #include "turtlelib/rigid2d.hpp"
// #include <ros/console.h>

// static nuturtlebot_msgs::WheelCommands wheel_cmd;
// static sensor_msgs::JointState js;
// static double motor_cmd_to_rad_sec, encoder_ticks_to_rad;
// static double left_encoder_ticks{0.0}, right_encoder_ticks{0.0}, left_wheel_angle{0.0}, right_wheel_angle{0.0}, left_wheel_velocity{0.0}, right_wheel_velocity{0.0};
// static turtlelib::Twist2D V;
// static turtlelib::Wheels_vel w_vel;
// static turtlelib::DiffDrive diff_drive;



// TEST_CASE("test turtle_interface","add_test"){
    
//     ros::NodeHandle nh;



//     //subscriber
//     ros::Subscriber cmd_vel_sub = nh.subscribe("red/wheel_cmd", 10, wheel_cmd_callback);
//     ros::Subscriber sensor_data_sub = nh.subscribe("red/sensor_data", 10, sensor_data_callback);
 
//     //publisher
//     ros::Publisher wheel_cmd_pub = nh.advertise<nuturtlebot_msgs::WheelCommands>("red/wheel_cmd", 500);
//     ros::Publisher joint_state_pub = nh.advertise<sensor_msgs::JointState>("red/joint_states", 500);

//     nh.getParam("motor_cmd_to_radsec", motor_cmd_to_rad_sec);

//     nh.getParam("encoder_ticks_to_rad", encoder_ticks_to_rad);


  
//     js.position = {1, 2};
//     js.velocity = {2, 3};


//     ros::Rate r(500);

//     SECTION(""){
//         d
//     }

// }

//publish, sleep, spinonce, check

