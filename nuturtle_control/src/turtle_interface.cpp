#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include "nuturtlebot_msgs/WheelCommands.h"
#include "nuturtlebot_msgs/SensorData.h"
#include <sensor_msgs/JointState.h>
#include "turtlelib/diff_drive.hpp"
#include "turtlelib/rigid2d.hpp"
#include <ros/console.h>

static nuturtlebot_msgs::WheelCommands wheel_cmd;
static sensor_msgs::JointState js;
static double motor_cmd_to_rad_sec, encoder_ticks_to_rad;
static double left_encoder_ticks{0.0}, right_encoder_ticks{0.0}, left_wheel_angle{0.0}, right_wheel_angle{0.0}, left_wheel_velocity{0.0}, right_wheel_velocity{0.0};


void cmd_vel_callback(const geometry_msgs::Twist::ConstPtr& twist_msg){
    //forward kinematics to get twist
    
    
    turtlelib::Twist2D V;
    turtlelib::Wheels_vel w_vel;
    
    V.x_dot = twist_msg->linear.x;
    V.theta_dot = twist_msg->angular.z;

    turtlelib::DiffDrive diff_drive;
    w_vel = diff_drive.inverse_kinematics(V);

    wheel_cmd.left_velocity = w_vel.w1_vel;
    wheel_cmd.right_velocity = w_vel.w2_vel;
    ROS_WARN("wheel_cmd left velocity: %f", wheel_cmd.left_velocity);
    ROS_WARN("wheel_cmd right velocity: %f", wheel_cmd.right_velocity);




}

void sensor_data_callback(const nuturtlebot_msgs::SensorData& sensor_msg){
    left_encoder_ticks  = sensor_msg.left_encoder; // encoder data in ticks
    right_encoder_ticks = sensor_msg.right_encoder;
    
    

    left_wheel_angle = (encoder_ticks_to_rad)*left_encoder_ticks;
    right_wheel_angle = (encoder_ticks_to_rad)*right_encoder_ticks;
    // ROS_INFO_STREAM("got angles");

    left_wheel_velocity = (motor_cmd_to_rad_sec)*left_encoder_ticks;
    right_wheel_velocity = (motor_cmd_to_rad_sec)*right_encoder_ticks;
    // ROS_INFO_STREAM("got velocity");

    js.header.stamp = ros::Time::now();
    // ROS_INFO_STREAM("got header stamp");
    js.name = {"red-wheel_left_joint", "red-wheel_right_joint"};
    // ROS_INFO_STREAM("got name");
    js.position = {left_wheel_angle, right_wheel_angle};
    // ROS_INFO_STREAM("got position");
    js.velocity = {left_wheel_velocity, right_wheel_velocity};
    // ROS_INFO_STREAM("got velocity");


}







int main(int argc, char **argv){

    ros::init(argc, argv, "turtle_interface");
    ros::NodeHandle nh;



    //subscriber
    ros::Subscriber cmd_vel_sub = nh.subscribe("cmd_vel", 1000, cmd_vel_callback);
    ros::Subscriber sensor_data_sub = nh.subscribe("red/sensor_data", 1000, sensor_data_callback);
 
    //publisher
    ros::Publisher wheel_cmd_pub = nh.advertise<nuturtlebot_msgs::WheelCommands>("red/wheel_cmd", 100);
    ros::Publisher joint_state_pub = nh.advertise<sensor_msgs::JointState>("red/joint_states", 100);

    nh.getParam("motor_cmd_to_radsec", motor_cmd_to_rad_sec);

    nh.getParam("encoder_ticks_to_rad", encoder_ticks_to_rad);


  
    js.position = {1, 2};
    js.velocity = {2, 3};


    ros::Rate r(500);


    //Twist forward_kinematics

    while(ros::ok()){
        ros::spinOnce();


        wheel_cmd_pub.publish(wheel_cmd);
        joint_state_pub.publish(js);



        r.sleep();

    }
    



}