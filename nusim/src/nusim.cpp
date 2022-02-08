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

static std_msgs::UInt64 timestep;
static double x, y, theta, rate, radius;
static int num_markers;
std::vector <double> x_s;
std::vector <double> y_s;
static double left_wheel_velocity {0.0};
static double right_wheel_velocity {0.0};
static double encoder_ticks_to_rad;



//sensor_data_message
static nuturtlebot_msgs::SensorData sensor_data;


bool reset_fn(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
    timestep.data = 0;
    x = 0;
    y = 0;
    return true;
}

bool teleport_fn(nusim::Teleport::Request &req, nusim::Teleport::Response &res){
    x = req.x;
    y = req.y;
    theta = req.theta;
    return true;

}


void wheel_cmd_callback(const nuturtlebot_msgs::WheelCommands::ConstPtr& msg){
    /// \brief set wheel velocities of the robot
    ///
    /// \param msg - nuturtlebot_msgs/WheelCommands

    left_wheel_velocity = msg->left_velocity;
    right_wheel_velocity = msg->right_velocity;
    // ROS_INFO_STREAM("got velocities");

    turtlelib::Wheel_angles wheel_angle;

    // left_wheel_velocity_rad = left_wheel_velocity*motor_cmd_to_radsec;
    // right_wheel_velocity_rad = right_wheel_velocity*motor_cmd_to_radsec;
    sensor_data.left_encoder = ((left_wheel_velocity)/rate + wheel_angle.w_ang1)/encoder_ticks_to_rad;
    sensor_data.right_encoder = ((right_wheel_velocity)/rate + wheel_angle.w_ang2)/encoder_ticks_to_rad;

    // ROS_INFO_STREAM("got encoder data");



}



int main(int argc, char ** argv){
    
    
    ros::init(argc, argv, "nusim");
    ros::NodeHandle nh;
    ros::NodeHandle nh2;

    ros::Publisher pub = nh.advertise<std_msgs::UInt64>("timestep_topic", 100);
    // ros::Publisher pub2 = nh2.advertise<sensor_msgs::JointState>("red/joint_states", 100);
    // ros::Publisher pub3 = nh.advertise<nusim::SensorData("red/sensor_data", 1000);


    ros::ServiceServer reset_service = nh.advertiseService("reset", reset_fn);
    // ros::Subscriber sub = nh.subscribe("geometry_msgs/Pose", 1000, pose_callback)
    ros::ServiceServer teleport_service = nh.advertiseService("teleport", teleport_fn);
    sensor_msgs::JointState red_joint_state;

    ros::Publisher vis_pub = nh.advertise<visualization_msgs::MarkerArray>("obstacles", 100, true);


    //subsribe to red/wheel_cmd
    ros::Subscriber wheel_cmd_sub = nh.subscribe<nuturtlebot_msgs::WheelCommands>("red/wheel_cmd", 1000, wheel_cmd_callback);
    

    //publish to red/sensor_data
    ros::Publisher sensor_pub = nh.advertise<nuturtlebot_msgs::SensorData>("red/sensor_data", 100);

    static tf2_ros::TransformBroadcaster broadcaster;



    //get parameters
    nh.param("x", x, 0.0);
    nh.param("y", y, 0.0);
    nh.param("theta", theta, 0.0);
    nh.param("rate", rate, 500.0);
    nh.getParam("num_markers", num_markers);
    nh.getParam("x_s", x_s);
    nh.getParam("y_s", y_s);
    nh.getParam("radius", radius);
    nh.getParam("encoder_ticks_to_rad", encoder_ticks_to_rad);



    // nh.param("x_m", x_m, 1);



    timestep.data = 0;
    visualization_msgs::MarkerArray marker_array;
        for (int i = 0; i < 3; i++){
            visualization_msgs::Marker marker;
            marker.header.frame_id = "world";
            marker.header.stamp = ros::Time::now();
            marker.ns = "obstacles";
            marker.id = i;
            marker.type = visualization_msgs::Marker:: CYLINDER;
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.position.x = x_s.at(i);
            marker.pose.position.y = y_s.at(i);
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
            
            // vis_pub.publish(marker);
            marker_array.markers.push_back(marker);
            // vis_pub.publish(marker_array);




        }
    vis_pub.publish(marker_array);
    ROS_INFO_STREAM("publishing markers");



    ros::Rate r(rate);

    while(ros::ok){

        pub.publish(timestep);
        ros::spinOnce();
        // ROS_INFO("%ld", timestep.data);
        timestep.data++;

        // red_joint_state.name.push_back("wheel_joint_1");
        // red_joint_state.name.push_back("wheel_joint_2");
        // red_joint_state.position.push_back(0);
        // red_joint_state.position.push_back(0);
        // red_joint_state.velocity.push_back(0);
        // red_joint_state.velocity.push_back(0);


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

        sensor_pub.publish(sensor_data);




        r.sleep();
    }
    return 0;
}

