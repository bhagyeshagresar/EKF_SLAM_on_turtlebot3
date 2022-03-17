/// \file the odometry node broadcasts the transform between odom and the blue-base footprint. The odometry node also 
/// publishes odometry messages on the odom topic
/// subscriber: topics - red/joint_states
/// publisher: topics - /odom




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

static nav_msgs::Odometry odom;
static turtlelib::Configuration current_config;
static turtlelib::Wheel_angles wheel_angle;
static turtlelib::Wheels_vel wheel_vel;   
static turtlelib::Twist2D V_twist;
static turtlelib::DiffDrive fwd_diff_drive;
static std::vector <double> positions;
static std::vector <double> velocities;


/// \brief function to compute the wheel_angles and wheel_velocities from joint_state message
/// \param js_msg - joint state message
void joint_state_callback(const sensor_msgs::JointState::ConstPtr&  js_msg){

    positions.resize(2);
    velocities.resize(2);
    positions = js_msg->position;
    velocities = js_msg->velocity;

    wheel_angle.w_ang1 = positions[0]; //wheel angle1
    wheel_angle.w_ang2 = positions[1]; // wheel angle2
    // wheel_angle.w_ang1 = 1.0;
    // wheel_angle.w_ang2 = 2.0;
    current_config = fwd_diff_drive.forward_kinematics(wheel_angle);

    // ROS_WARN("wheel angles");

    // current_config = fwd_diff_drive.forward_kinematics(wheel_angle);
    


    wheel_vel.w1_vel = velocities[0]; //wheel velocity 1
    wheel_vel.w2_vel = velocities[1]; //wheel velocity 2
    // wheel_vel.w1_vel = 3.0;
    // wheel_vel.w2_vel = 4.0;
    ROS_WARN("wheel_vel.w1_vel: %f", wheel_vel.w1_vel);
    ROS_WARN("wheel_vel.w2_vel: %f", wheel_vel.w2_vel);


    // twist comes from wheel velocities in joint states

    V_twist.x_dot = (((fwd_diff_drive.get_radius()*(wheel_vel.w1_vel +  wheel_vel.w2_vel))/2));
    V_twist.theta_dot = (((fwd_diff_drive.get_radius()*(wheel_vel.w2_vel -  wheel_vel.w1_vel))/(2*fwd_diff_drive.get_length_d())));

    ROS_WARN("forward twist: %f", V_twist.x_dot);
    ROS_WARN("forward twist: %f", V_twist.theta_dot);




}

/// \brief function to set the pose of the blue robot
/// \param req - x, y and theta configuration values of the robot
/// \param res - empty response message 
bool set_pose(nuturtle_control::Set_Pose::Request &req, nuturtle_control::Set_Pose::Response &res){
    

    current_config.x_config = req.x_config;
    current_config.y_config = req.y_config;
    current_config.theta_config = req.theta_config;

    fwd_diff_drive.set_config(current_config);


    return true;

}



int main(int argc, char **argv){

    //initialize rosnode odometry
    ros::init(argc, argv, "odometry");

    //create nodehandle object
    ros::NodeHandle nh;


    

    //subscribe to red/jointStates
    ros::Subscriber js_sub = nh.subscribe("red/joint_states", 10, joint_state_callback);


    //publish on odom topic
    ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 500);
    
    //provide service set_pose
    ros::ServiceServer service = nh.advertiseService("set_pose", set_pose);

    //transform between odom and blue-base_footprint
    tf2_ros::TransformBroadcaster odom_broadcaster;






    ros::Time current_time;
	ros::Time last_time;
	current_time = ros::Time::now();
	last_time = ros::Time::now();

    geometry_msgs::TransformStamped odom_trans;


    ros::Rate r(500);


    
    
    while(ros::ok()){
        current_time = ros::Time::now();
        



    
        //calculate twist
        // V_twist.x_dot = (fwd_diff_drive.get_radius()*(wheel_vel.w1_vel + wheel_vel.w2_vel))/2;
        // V_twist.theta_dot = (fwd_diff_drive.get_radius()*(wheel_vel.w2_vel - wheel_vel.w1_vel))/(2*fwd_diff_drive.get_length_d());



        //get the current configuration of the blue robot
        current_config = fwd_diff_drive.get_config();
      
        //publish transform between odom and blue-base_footprint on tf
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

       
        //set the header.stamp and header.frame_id for the odometry message
        nav_msgs::Odometry odom;
        odom.header.stamp = current_time;
        odom.header.frame_id = "odom";

        //set the position for the robot
        odom.pose.pose.position.x = current_config.x_config;
        odom.pose.pose.position.y = current_config.y_config;
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation.x = quat.x();
        odom.pose.pose.orientation.y = quat.y();
        odom.pose.pose.orientation.z = quat.z();
        odom.pose.pose.orientation.w = quat.w();


        //set the velocity of the robot
        odom.child_frame_id = "blue-base_footprint";
        odom.twist.twist.linear.x = V_twist.x_dot;
        odom.twist.twist.linear.y = 0;
        odom.twist.twist.angular.z = V_twist.theta_dot;
        //publish the odometry message
        odom_pub.publish(odom);


        
        last_time = current_time;
        ros::spinOnce();

        r.sleep();    
    
    
    
    }

}