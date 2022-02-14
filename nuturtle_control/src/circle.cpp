/// \file
/// \brief the circle node publishes cmd_vel messages which is a twist message to drive the turtlebot in a circle
/// publisher: geometry_msgs/Twist message on cmd_vel topic


#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include "nuturtle_control/Control.h"
#include <std_srvs/Empty.h>
#include <ros/console.h>


//define the variables
static geometry_msgs::Twist twist;
static double angular_velocity{0.6};
static double turning_radius{0.2};
static bool stop = false;
ros::Publisher cmd_vel_pub;

/// \brief function to change the angular velocity and radius of the circle in which the robot moves
/// \param req - angular velocity and radius
/// \param res - empty response
bool control_fn(nuturtle_control::Control::Request &req, nuturtle_control::Control::Response &res){
    angular_velocity = 0.0;
    turning_radius = 0.0;
    angular_velocity = req.velocity; // velocity in rad/s, positive means counter-clockwise, negative_clockwise
    turning_radius = req.radius;
    stop = false;

    return true;
}


/// \brief function to move the robot in the reverse direction
/// \param req - empty request message
/// \param res - empty response message
bool reverse_fn(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
    angular_velocity = -(angular_velocity);
    return true;
}


/// \brief function to stop the robot. nusim publishes a zero twist message. stop flag set to true indicates 
/// turlebot should not move and the cmd_vel publisher publishes zero twist
/// \param req - empty request message
/// \param res - empty response message
bool stop_fn(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
    stop = true;
    twist.linear.x = 0.0;
    twist.angular.z = 0.0;
    cmd_vel_pub.publish(twist);

    return true;
}


int main(int argc, char **argv){
    

    //initialize rosnode circle
    ros::init(argc, argv, "circle");
    
    //create nodehandle object
    ros::NodeHandle nh;


    //advertise geometry_msgs/Twist on cmd_vel topic
    cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 500);
    
    //advertise control service called "control"
    ros::ServiceServer control_service = nh.advertiseService("control", control_fn);

    //advertise reverse service called "reverse"
    ros::ServiceServer reverse_service = nh.advertiseService("reverse", reverse_fn);
    
    //advertise stop service called "stop"
    ros::ServiceServer stop_service = nh.advertiseService("stop", stop_fn);




    ros::Rate r(500);



    while(ros::ok()){
        
        //calculate the linear velocity and angular velocity for the twist        
        twist.linear.x = turning_radius*angular_velocity;
        twist.angular.z = angular_velocity;


        //set stop flag. stop flag == false indicates the robot will keep publishing a twist message.
        if(stop == false){
            cmd_vel_pub.publish(twist);

        }
        ros::spinOnce();


        r.sleep();

    }





}