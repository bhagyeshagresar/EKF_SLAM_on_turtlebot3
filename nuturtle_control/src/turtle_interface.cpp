/// \file the turtle_interface node subscribes to cmd_vel topic. The turtle_interface node controls the turtlebot 
/// with the help of the geometry_msgs/Twist messagw which it subscribes to. The turtle_interface also publishes wheel_cmd
/// left and right wheel velocities in ticks. The turle_interface node also publishes joint_states to red/joint_states topic.
/// Publisher : topics : red/wheel_cmd, red/joint_states
/// Subscriber : topics : cmd_vel, red/sensor_data






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
static int left_encoder_ticks{0}, right_encoder_ticks{0};
static double left_wheel_angle{0.0}, right_wheel_angle{0.0}, left_wheel_velocity{0.0}, right_wheel_velocity{0.0};
static turtlelib::Twist2D V;
static turtlelib::Wheels_vel w_vel;
static turtlelib::DiffDrive diff_drive;


/// \brief function to calculate left and right wheel velocites and keep it in range between -256 and 256
/// \param twist_msg - linear and angular velocity
void cmd_vel_callback(const geometry_msgs::Twist::ConstPtr& twist_msg){
    
    
    
    V.x_dot = twist_msg->linear.x;
    V.theta_dot = twist_msg->angular.z;

    w_vel = diff_drive.inverse_kinematics(V);

    wheel_cmd.left_velocity = (int)(w_vel.w1_vel/motor_cmd_to_rad_sec);
    wheel_cmd.right_velocity = (int)(w_vel.w2_vel/motor_cmd_to_rad_sec);


    if(wheel_cmd.left_velocity < -256){
        wheel_cmd.left_velocity = -256;
    }

    if(wheel_cmd.left_velocity > 256){
        wheel_cmd.left_velocity = 256;
    }

    if(wheel_cmd.right_velocity > 256){
        wheel_cmd.right_velocity = 256;
    }
    
    if(wheel_cmd.right_velocity < -256){
        wheel_cmd.right_velocity = -256;
    }


}

/// \brief function to get joint states for left and right wheels. Function calculates the left and right encoder ticks
/// and from that left and right wheel angles and velocities are calculated using the parameters defined in the diff_drive.yaml file
/// \param sensor_msg - left and right encoder ticks message from nusim
void sensor_data_callback(const nuturtlebot_msgs::SensorData& sensor_msg){
    left_encoder_ticks  = sensor_msg.left_encoder; // encoder data in ticks
    right_encoder_ticks = sensor_msg.right_encoder;
    
    

    left_wheel_angle = (double)(encoder_ticks_to_rad)*left_encoder_ticks;
    right_wheel_angle = (double)(encoder_ticks_to_rad)*right_encoder_ticks;

    left_wheel_velocity = (double)(motor_cmd_to_rad_sec)*left_encoder_ticks;
    right_wheel_velocity = (double)(motor_cmd_to_rad_sec)*right_encoder_ticks;

    js.header.stamp = ros::Time::now();
    js.name = {"red-wheel_left_joint", "red-wheel_right_joint"};
    js.position = {left_wheel_angle, right_wheel_angle};
    js.velocity = {left_wheel_velocity, right_wheel_velocity};
    // js.position = {1, 2};
    // js.velocity = {2, 3};
    js.effort = {0.0, 0.0};


}




int main(int argc, char **argv){

    //initialise rosnode turtle_interface
    ros::init(argc, argv, "turtle_interface");
    
    //create rosnode handle object
    ros::NodeHandle nh;



    //subscribe to cmd_vel topic
    ros::Subscriber cmd_vel_sub = nh.subscribe("cmd_vel", 10, cmd_vel_callback);
    
    //subscribe to red/sensor_data topic
    ros::Subscriber sensor_data_sub = nh.subscribe("red/sensor_data", 10, sensor_data_callback);
 
    //advertise nuturtlebot_msgs/WheelCommands message on red/wheel_cmd topic
    ros::Publisher wheel_cmd_pub = nh.advertise<nuturtlebot_msgs::WheelCommands>("red/wheel_cmd", 500);
    
    //advertise sensor_msgs/JointState message on red/joint_states
    ros::Publisher joint_state_pub = nh.advertise<sensor_msgs::JointState>("red/joint_states", 500);

    
    //get the parameters from diff_drive.yaml
    nh.getParam("motor_cmd_to_radsec", motor_cmd_to_rad_sec);
    nh.getParam("encoder_ticks_to_rad", encoder_ticks_to_rad);


  
    // js.position = {1, 2};
    // js.velocity = {2, 3};
    // js.name = {"red-wheel_left_joint", "red-wheel_right_joint"};
    // js.position = {left_wheel_angle, right_wheel_angle};
    // js.velocity = {left_wheel_velocity, right_wheel_velocity};


    ros::Rate r(500);



    while(ros::ok()){

        //publish wheel_cmd(left and right velocities in ticks on the red/wheel_cmd topic)
        wheel_cmd_pub.publish(wheel_cmd);

        //publish joint states on the red/joint_states topic
        joint_state_pub.publish(js);

        ros::spinOnce();
        r.sleep();

    }
    



}