/// \file
/// \brief nusim node subscribes to wheel_cmd topic and published sensor_data message. The nusim node 
/// provides a simulation for the robot and other objects in the environment. The node provides a reset service
/// and teleport service to reset the simulation to initial state and to teleport the turtlebot to a specific location
/// subscriber topic: red/wheel_cmd
/// publisher topics: red/sensor_data, /obstacles, /walls


#include <ros/ros.h> 
#include <std_msgs/UInt64.h>
#include <ros/console.h>
#include <iostream>
#include <std_srvs/Empty.h>
#include <sensor_msgs/JointState.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include "nusim/Teleport.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include <vector>
#include "nuturtlebot_msgs/WheelCommands.h"
#include "nuturtlebot_msgs/SensorData.h"
#include "turtlelib/diff_drive.hpp"


/// Define the variables
static std_msgs::UInt64 timestep;
static double x, y, theta, rate, radius;
static int num_markers;
std::vector <double> x_m;
std::vector <double> y_m;
std::vector <double> wall_xpos{3.0, -3.0, 0.0, 0.0};
std::vector <double> wall_ypos{0.0, 0.0, 3.0, -3.0};
std::vector <double> x_length;
std::vector <double> y_length;


static double encoder_ticks_to_rad{0.0}, motor_cmd_to_radsec{0.0};
static int wheel_velocity_left {0};
static int wheel_velocity_right {0};

static turtlelib::DiffDrive update_config;
static turtlelib::Wheels_vel wheel_velocities;
static turtlelib::Wheel_angles wheel_angle;
static turtlelib::Configuration current_config;


//sensor_data_message
static nuturtlebot_msgs::SensorData sensor_data;


/// \brief function to reset to the initial state of the simulation
/// \param req - empty request message
/// \param res - empty response message 
bool reset_fn(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
    timestep.data = 0;
    x = 0;
    y = 0;
    return true;
}



/// \brief function to teleport the robot to a specific location
/// \param req - the request message takes float64 x, y and theta
/// \param res - the response message is an empty response
bool teleport_fn(nusim::Teleport::Request &req, nusim::Teleport::Response &res){
    x = req.x;
    y = req.y;
    theta = req.theta;
    return true;

}

/// \brief nusim node subscribes to red/wheel_cmd topic. The callback function set wheel velocities of the robot
/// and also calculates encoder ticks for left and right wheels
/// \param msg - nuturtlebot_msgs/WheelCommands 
void wheel_cmd_callback(const nuturtlebot_msgs::WheelCommands::ConstPtr& msg){
   

    wheel_velocities.w1_vel = (wheel_velocity_left*0.024);
    wheel_velocities.w2_vel = (wheel_velocity_right*0.024);

   

    sensor_data.left_encoder = (int)((wheel_velocities.w1_vel/rate) + wheel_angle.w_ang1)/encoder_ticks_to_rad;
    sensor_data.right_encoder = (int)((wheel_velocities.w2_vel/rate) + wheel_angle.w_ang2)/encoder_ticks_to_rad;

   
   
    
}



int main(int argc, char ** argv){
    
    //initialise rosnode nusim
    ros::init(argc, argv, "nusim");

    //create nodehandle objects
    ros::NodeHandle nh;
    ros::NodeHandle nh2;

    //publish to timestep topic
    ros::Publisher pub = nh.advertise<std_msgs::UInt64>("timestep_topic", 500);
   
    //advertise the reset service
    ros::ServiceServer reset_service = nh.advertiseService("reset", reset_fn);
    
    //advertise the teleport service
    ros::ServiceServer teleport_service = nh.advertiseService("teleport", teleport_fn);
    sensor_msgs::JointState red_joint_state;

    //publisher for cylindrical obstacles
    ros::Publisher vis_pub = nh.advertise<visualization_msgs::MarkerArray>("obstacles", 500, true);

    //walls publisher
    ros::Publisher wall_pub = nh.advertise<visualization_msgs::MarkerArray>("walls", 500, true);



    //subsribe to red/wheel_cmd
    ros::Subscriber wheel_cmd_sub = nh.subscribe<nuturtlebot_msgs::WheelCommands>("red/wheel_cmd", 10, wheel_cmd_callback);
    

    //publish to red/sensor_data
    ros::Publisher sensor_pub = nh.advertise<nuturtlebot_msgs::SensorData>("red/sensor_data", 500);

    static tf2_ros::TransformBroadcaster broadcaster;



    //get parameters from basic_world.yaml
    nh.getParam("x0", x);
    nh.getParam("y0", y);
    nh.param("theta0", theta);
    nh.param("rate", rate, 500.0);
    nh.getParam("num_markers", num_markers);
    nh.getParam("x_m", x_m);
    nh.getParam("y_m", y_m);
    nh.getParam("radius", radius);
    nh.getParam("encoder_ticks_to_rad", encoder_ticks_to_rad);
    nh.getParam("motor_cmd_to_rad_sec", motor_cmd_to_radsec);
    nh.getParam("x_length", x_length);
    nh.getParam("y_length", y_length);



    //visualize the walls
    visualization_msgs::MarkerArray wall_array;
        for(int j = 0; j < 4; j++){
            visualization_msgs::Marker wall_marker;
            wall_marker.header.frame_id = "world";
            wall_marker.header.stamp = ros::Time::now();
            wall_marker.id = j;
            wall_marker.type = visualization_msgs::Marker::CUBE;
            wall_marker.action = visualization_msgs::Marker::ADD;
            wall_marker.pose.position.x = wall_xpos.at(j);
            wall_marker.pose.position.y = wall_ypos.at(j);
            wall_marker.pose.position.z = 0;
            wall_marker.pose.orientation.x = 0.0;
            wall_marker.pose.orientation.y = 0.0;
            wall_marker.pose.orientation.z = 0.0;
            wall_marker.pose.orientation.w = 1.0;
            wall_marker.scale.x = x_length.at(j);
            wall_marker.scale.y = y_length.at(j);
            wall_marker.scale.z = 0.25;
            wall_marker.color.a = 1.0;
            wall_marker.color.r = 1.0;
            wall_marker.color.g = 0.0;
            wall_marker.color.b = 0.0;
            wall_marker.lifetime = ros::Duration();
            
            wall_array.markers.push_back(wall_marker);




        }
    wall_pub.publish(wall_array);


        


    //visualize the cylindrical obstacles
    timestep.data = 0;
    visualization_msgs::MarkerArray marker_array;
        for (int i = 0; i < num_markers; i++){
            visualization_msgs::Marker marker;
            marker.header.frame_id = "world";
            marker.header.stamp = ros::Time::now();
            marker.ns = "obstacles";
            marker.id = i;
            marker.type = visualization_msgs::Marker:: CYLINDER;
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.position.x = x_m.at(i);
            marker.pose.position.y = y_m.at(i);
            marker.pose.position.z = 0;
            marker.pose.orientation.x = 0.0;
            marker.pose.orientation.y = 0.0;
            marker.pose.orientation.z = 0.0;
            marker.pose.orientation.w = 1.0;
            marker.scale.x = radius*2;
            marker.scale.y = radius*2;
            marker.scale.z = 0.25;
            marker.color.a = 1.0;
            marker.color.r = 1.0;
            marker.color.g = 0.0;
            marker.color.b = 0.0;
            ROS_INFO("check for rviz");
            marker.lifetime = ros::Duration();
            
            marker_array.markers.push_back(marker);




        }
    vis_pub.publish(marker_array);
    ROS_INFO_STREAM("publishing markers");



    ros::Rate r(rate);

    while(ros::ok){

        pub.publish(timestep);
        timestep.data++;

        //calculate the wheel angles using wheel velocities
        wheel_angle.w_ang1 = ((wheel_velocities.w1_vel/rate) + wheel_angle.w_ang1);
        wheel_angle.w_ang2 = ((wheel_velocities.w2_vel/rate) + wheel_angle.w_ang2);

       //Get the current_configuration of the robot using forward kinematics function
        current_config = update_config.forward_kinematics(wheel_angle);
        
        

        x = current_config.x_config;
        y = current_config.y_config;
        theta = current_config.theta_config;

       
        //broadcast the transform between nusim and red-base_footprint
        geometry_msgs::TransformStamped transformStamped;
        transformStamped.header.stamp = ros::Time::now();
        transformStamped.header.frame_id = "world";
        transformStamped.child_frame_id = "red-base_footprint";
        transformStamped.transform.translation.x = x;
        transformStamped.transform.translation.y = y;
        transformStamped.transform.translation.z = 0;
        tf2::Quaternion q;
        q.setRPY(0, 0, theta);
        transformStamped.transform.rotation.x = q.x();
        transformStamped.transform.rotation.y = q.y();
        transformStamped.transform.rotation.z = q.z();
        transformStamped.transform.rotation.w = q.w();
        broadcaster.sendTransform(transformStamped);

       
       

        //publish sensor_data on red/sensor_data topic
        sensor_pub.publish(sensor_data);


        ros::spinOnce();


        r.sleep();
    }
    return 0;
}

