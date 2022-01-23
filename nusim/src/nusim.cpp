#include <ros/ros.h>
#include <std_msgs/UInt64.h>
#include <ros/console.h>
#include <iostream>
#include <std_srvs/Empty.h>
#include <sensor_msgs/JointState.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/Pose.h>
#include "nusim/Teleport.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include <vector>

static std_msgs::UInt64 timestep;
static double x, y, theta, rate, radius;
static int num_markers;
// static double x_m[], y_m[];
std::vector <double> x_m;
std::vector <double> y_m;



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





int main(int argc, char ** argv){
    
    
    ros::init(argc, argv, "nusim");
    ros::NodeHandle nh("~");
    ros::NodeHandle nh2;

    ros::Publisher pub = nh.advertise<std_msgs::UInt64>("timestep_topic", 100);
    ros::Publisher pub2 = nh2.advertise<sensor_msgs::JointState>("red/joint_states", 100);


    ros::ServiceServer reset_service = nh.advertiseService("reset", reset_fn);
    // ros::Subscriber sub = nh.subscribe("geometry_msgs/Pose", 1000, pose_callback)
    ros::ServiceServer teleport_service = nh.advertiseService("teleport", teleport_fn);
    sensor_msgs::JointState red_joint_state;

    ros::Publisher vis_pub = nh.advertise<visualization_msgs::MarkerArray>("obstacles", 100, true);

    //get parameters
    nh.param("x", x, 0.0);
    nh.param("y", y, 0.0);
    nh.param("theta", theta, 0.0);
    nh.param("rate", rate, 500.0);
    nh.getParam("num_markers", num_markers);
    nh.getParam("x_s", x_m);
    nh.getParam("y_s", y_m);
    nh.getParam("radius", radius);



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
            
            // vis_pub.publish(marker);
            marker_array.markers.push_back(marker);
            // vis_pub.publish(marker_array);




        }
    vis_pub.publish(marker_array);



    ros::Rate r(rate);

    while(ros::ok){

        pub.publish(timestep);
        ros::spinOnce();
        // ROS_INFO("%ld", timestep.data);
        timestep.data++;

        red_joint_state.name.push_back("wheel_joint_1");
        red_joint_state.name.push_back("wheel_joint_2");
        red_joint_state.position.push_back(0);
        red_joint_state.position.push_back(0);
        red_joint_state.velocity.push_back(0);
        red_joint_state.velocity.push_back(0);


        static tf2_ros::StaticTransformBroadcaster static_broadcaster;
        geometry_msgs::TransformStamped static_transformStamped;
        static_transformStamped.header.stamp = ros::Time::now();
        static_transformStamped.header.frame_id = "world";
        static_transformStamped.child_frame_id = "red:base_footprint";
        static_transformStamped.transform.translation.x = x;
        static_transformStamped.transform.translation.y = y;
        static_transformStamped.transform.translation.z = 0;
        tf2::Quaternion q;
        q.setRPY(0, 0, theta);
        static_transformStamped.transform.rotation.x = q.x();
        static_transformStamped.transform.rotation.y = q.y();
        static_transformStamped.transform.rotation.z = q.z();
        static_transformStamped.transform.rotation.w = q.w();
        static_broadcaster.sendTransform(static_transformStamped);

       





        r.sleep();
    }
    return 0;
}

