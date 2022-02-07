#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <nav_msgs/Odometry.h>
#include <tf2_ros/transform_broadcaster.h>
#include "nuturtle_control/set_pose.h"
#include <std_srvs/Empty.h>
#include "turtlelib/diff_drive.hpp"
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>

nav_msgs::Odometry odom;
static turtlelib::Configuration current_config;
static turtlelib::Wheel_angles wheel_angle;
static turtlelib::Wheels_vel wheel_vel;   
    

void joint_state_callback(const sensor_msgs::JointState::ConstPtr&  js_msg){
    
    
    wheel_angle.w_ang1 = js_msg->position[0]; //wheel angle1
    wheel_angle.w_ang2 = js_msg->position[1]; // wheel angle2
    
    wheel_vel.w1_vel = js_msg->velocity[0]; //wheel velocity 1
    wheel_vel.w2_vel = js_msg->velocity[1]; //wheel velocity 2
    
}


bool set_pose(nuturtle_control::set_pose::Request &req, std_srvs::Empty::Response &res){
    

    odom.pose.pose.position.x = req.x_config;
    odom.pose.pose.position.y = req.y_config;
    odom.pose.pose.position.z = req.theta_config;
    // odom.pose.pose.orientation = odom_quat;

    return true;

}



int main(int argc, char **argv){
    
    ros::init(argc, argv, "odometry");
    ros::NodeHandle nh;


    //get the parameters
    // nh.getParam("body_id", body_id);
    // nh.getParam("odom_id", odom_id);
    // nh.getParam("wheel_left", wheel_left);
    // nh.getParam("wheel_right", wheel_right);


    //subscribe to jointStates
    ros::Subscriber js_sub = nh.subscribe("joint_states", 1000, joint_state_callback);


    //publish on odom topic
    ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 100);
    
    //provide service
    ros::ServiceServer service = nh.advertiseService("set_pose", set_pose);


    tf2_ros::TransformBroadcaster odom_broadcaster;





    // initial position of robot at origin of odom coordinate frame
	// double x = 0.0; 
	// double y = 0.0;
	// double th = 0;



    // // velocity - base_link moves in the odom_frame 
	// double vx = 0.1;
	// double vy = -0.1;
	// double vth = 0.1;


    ros::Time current_time;
	ros::Time last_time;
	current_time = ros::Time::now();
	last_time = ros::Time::now();

    ros::Rate r(1);

    
    
    while(ros::ok()){
        ros::spinOnce();
        current_time = ros::Time::now();
        

        //get the wheel angles
        turtlelib::DiffDrive fwd_diff_drive;

        current_config = fwd_diff_drive.forward_kinematics(wheel_angle);

        //get the twist
        turtlelib::Twist2D V_twist;


    

        V_twist.x_dot = (fwd_diff_drive.get_radius()*(wheel_vel.w1_vel + wheel_vel.w2_vel))/2;
        V_twist.theta_dot = (fwd_diff_drive.get_radius()*(wheel_vel.w2_vel - wheel_vel.w1_vel))/(2*fwd_diff_drive.get_length_d());


        
        // double dt = 0.01;
        // double delta_x = (vx * cos(th) - vy * sin(th)) * dt; // use forward_kinematics to
        // double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
        // double delta_th = vth * dt;
   
        // current_config.x += delta_x;
        // current_config.y += delta_y;
        // current_config.th += delta_th;

    

      
        //publish transform over tf
        geometry_msgs::TransformStamped odom_trans;
        odom_trans.header.stamp = current_time;
        odom_trans.header.frame_id = "odom_id";
        odom_trans.child_frame_id = "body_id";

    
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
        odom.header.frame_id = "odom_id";

        //set the position
        odom.pose.pose.position.x = current_config.x_config;
        odom.pose.pose.position.y = current_config.y_config;
        odom.pose.pose.position.z = current_config.theta_config;
        odom.pose.pose.orientation.x = quat.x();
        odom.pose.pose.orientation.y = quat.y();
        odom.pose.pose.orientation.z = quat.z();
        odom.pose.pose.orientation.w = quat.w();


        //set the velocity
        odom.child_frame_id = "body_id";
        odom.twist.twist.linear.x = V_twist.x_dot;
        odom.twist.twist.linear.y = 0;
        odom.twist.twist.angular.z = V_twist.theta_dot;

        odom_pub.publish(odom);


        
        last_time = current_time;
        r.sleep();    
    
    
    
    }

}