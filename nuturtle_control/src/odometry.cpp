#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/JointState.h>
#include <nav_msgs/Odometry.h>
#include <tf2_ros/transform_broadcaster.h>
#include "nuturtle_control/Set_Pose.h"
#include <std_srvs/Empty.h>
#include "turtlelib/diff_drive.hpp"
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>

nav_msgs::Odometry odom;
static turtlelib::Configuration current_config;
static turtlelib::Wheel_angles wheel_angle;
static turtlelib::Wheels_vel wheel_vel;   
static turtlelib::Twist2D V_twist;
static turtlelib::DiffDrive fwd_diff_drive;



void joint_state_callback(const sensor_msgs::JointState::ConstPtr&  js_msg){
    
    // ROS_INFO_STREAM("JOINT STATES RECEIVED");
    wheel_angle.w_ang1 = js_msg->position[0]; //wheel angle1
    wheel_angle.w_ang2 = js_msg->position[1]; // wheel angle2


    current_config = fwd_diff_drive.forward_kinematics(wheel_angle);
    

    // ROS_INFO_STREAM("JOINT POS RECEIVED");

    wheel_vel.w1_vel = js_msg->velocity[0]; //wheel velocity 1
    wheel_vel.w2_vel = js_msg->velocity[1]; //wheel velocity 2
    // ROS_INFO_STREAM("JOINT VELS RECEIVED");

}


bool set_pose(nuturtle_control::Set_Pose::Request &req, nuturtle_control::Set_Pose::Response &res){
    // current_config.x_config = 0.0;
    // current_config.y_config = 0.0;
    // current_config.theta_config = 0.0;


    current_config.x_config = req.x_config;
    current_config.y_config = req.y_config;
    current_config.theta_config = req.theta_config;

    fwd_diff_drive.set_config(current_config);







    return true;

}



int main(int argc, char **argv){
    // ROS_INFO_STREAM("Hello world");
    ros::init(argc, argv, "odometry");
    ros::NodeHandle nh;


    

    //subscribe to jointStates
    ros::Subscriber js_sub = nh.subscribe("red/joint_states", 10, joint_state_callback);


    //publish on odom topic
    ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 500);
    
    //provide service
    ros::ServiceServer service = nh.advertiseService("set_pose", set_pose);


    tf2_ros::TransformBroadcaster odom_broadcaster;






    ros::Time current_time;
	ros::Time last_time;
	current_time = ros::Time::now();
	last_time = ros::Time::now();

    geometry_msgs::TransformStamped odom_trans;


    ros::Rate r(500);


    
    
    while(ros::ok()){
        current_time = ros::Time::now();
        

        //get the wheel angles
        // turtlelib::DiffDrive fwd_diff_drive;

        // current_config = fwd_diff_drive.forward_kinematics(wheel_angle);

        //get the twist


    

        V_twist.x_dot = (fwd_diff_drive.get_radius()*(wheel_vel.w1_vel + wheel_vel.w2_vel))/2;
        V_twist.theta_dot = (fwd_diff_drive.get_radius()*(wheel_vel.w2_vel - wheel_vel.w1_vel))/(2*fwd_diff_drive.get_length_d());


        
        current_config = fwd_diff_drive.get_config();
      
        //publish transform over tf
        odom_trans.header.stamp = current_time;
        odom_trans.header.frame_id = "odom";
        odom_trans.child_frame_id = "blue-base_footprint";

    
        odom_trans.transform.translation.x = current_config.x_config;
        odom_trans.transform.translation.y = current_config.y_config;
        odom_trans.transform.translation.z = 0.0;
        tf2::Quaternion quat;
        quat.setRPY(0, 0, current_config.theta_config);
        odom_trans.transform.rotation.x = quat.x();
        odom_trans.transform.rotation.y = quat.y();
        odom_trans.transform.rotation.z = quat.z();
        odom_trans.transform.rotation.w = quat.w();
        //send the transform
        odom_broadcaster.sendTransform(odom_trans);

       
        //publish odometry message over ros
        nav_msgs::Odometry odom;
        odom.header.stamp = current_time;
        odom.header.frame_id = "odom";

        //set the position
        odom.pose.pose.position.x = current_config.x_config;
        odom.pose.pose.position.y = current_config.y_config;
        odom.pose.pose.position.z = current_config.theta_config;
        odom.pose.pose.orientation.x = quat.x();
        odom.pose.pose.orientation.y = quat.y();
        odom.pose.pose.orientation.z = quat.z();
        odom.pose.pose.orientation.w = quat.w();


        //set the velocity
        odom.child_frame_id = "blue-base_footprint";
        odom.twist.twist.linear.x = V_twist.x_dot;
        odom.twist.twist.linear.y = 0;
        odom.twist.twist.angular.z = V_twist.theta_dot;

        odom_pub.publish(odom);


        
        last_time = current_time;
        ros::spinOnce();

        r.sleep();    
    
    
    
    }

}