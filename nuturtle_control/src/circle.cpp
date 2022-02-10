#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include "nuturtle_control/Control.h"
#include <std_srvs/Empty.h>
#include <ros/console.h>



static geometry_msgs::Twist twist;
static double angular_velocity{6.0};
static double turning_radius{4.0};

static bool stop = false;
ros::Publisher cmd_vel_pub;


bool control_fn(nuturtle_control::Control::Request &req, nuturtle_control::Control::Response &res){
    angular_velocity = req.velocity; // velocity in rad/s, positive means counter-clockwise, negative_clockwise
    turning_radius = req.radius;


    return true;
}

bool reverse_fn(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
    angular_velocity = -(angular_velocity);
    return true;
}


bool stop_fn(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
    twist.linear.x = 0.0;
    twist.angular.z = 0.0;
    cmd_vel_pub.publish(twist);
    stop == true;

    return true;
}


int main(int argc, char **argv){
    
    ros::init(argc, argv, "circle");
    
    ros::NodeHandle nh;


    //Publish cmd_vel
    cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 100);
    
    ros::ServiceServer control_service = nh.advertiseService("control", control_fn);

    ros::ServiceServer reverse_service = nh.advertiseService("reverse", reverse_fn);
    
    ros::ServiceServer stop_service = nh.advertiseService("stop", stop_fn);




    ros::Rate r(500);



    while(ros::ok()){
        
        
        ros::spinOnce();

        
         
        
        twist.linear.x = turning_radius*angular_velocity;
        twist.angular.z = angular_velocity;
        if(stop == false){
            cmd_vel_pub.publish(twist);

        }
    
        // ROS_INFO_STREAM("twist %d, twist");

        r.sleep();

    }





}