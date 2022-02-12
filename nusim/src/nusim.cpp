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
std::vector <double> x_m;
std::vector <double> y_m;
std::vector <double> wall_xpos{3.0, -3.0, 0.0, 0.0};
std::vector <double> wall_ypos{0.0, 0.0, 3.0, -3.0};
std::vector <double> x_length;
std::vector <double> y_length;





// static double left_wheel_velocity {0.0}; //unused
// static double right_wheel_velocity {0.0};
static double encoder_ticks_to_rad{0.0}, motor_cmd_to_radsec{0.0};
static int wheel_velocity_left {0};
static int wheel_velocity_right {0};

static turtlelib::DiffDrive update_config;
static turtlelib::Wheels_vel wheel_velocities;
static turtlelib::Wheel_angles wheel_angle;
static turtlelib::Configuration current_config;




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
    
    // turtlelib::Wheels_vel wheel_velocities;
    // turtlelib::Wheel_angles wheel_angle;
    // turtlelib::Configuration current_config;
    
    
    // wheel_velocities.w1_vel = msg->left_velocity*motor_cmd_to_radsec; //motor cmd to rad/sec
    // wheel_velocities.w2_vel = msg->right_velocity*motor_cmd_to_radsec;

    ROS_WARN("nusim:msg->left_velocity %d", msg->left_velocity);
    ROS_WARN("nusim:msg->left_velocity %d", msg->right_velocity);

    
    wheel_velocity_left = msg->left_velocity; //motor cmd to rad/sec
    wheel_velocity_right = msg->right_velocity;


    ROS_WARN("nusim:wheel_velocity_left %d", wheel_velocity_left);
    ROS_WARN("nusim:wheel_velocity_right %d", wheel_velocity_right);

    // if(wheel_velocities.w1_vel < -256.0){
    //     wheel_velocities.w1_vel = -256.0;
    // }

    // else if(wheel_velocities.w1_vel > 256.0){
    //     wheel_velocities.w1_vel = 256.0;
    // }

    // else if(wheel_velocities.w2_vel > 256.0){
    //     wheel_velocities.w2_vel = 256.0;
    // }
    
    // else if(wheel_velocities.w2_vel < -256.0){
    //     wheel_velocities.w2_vel = -256.0;
    // }

    ROS_WARN("nusim:wheel_velocity.w1_vel before %f", wheel_velocities.w1_vel);
    ROS_WARN("nusim:wheel_velocity.w2_vel before %f", wheel_velocities.w2_vel);



    wheel_velocities.w1_vel = (wheel_velocity_left*0.024);//ticks
    wheel_velocities.w2_vel = (wheel_velocity_right*0.024);

    ROS_WARN("nusim:wheel_velocity.w1_vel after %f", wheel_velocities.w1_vel);
    ROS_WARN("nusim:wheel_velocity.w2_vel after %f", wheel_velocities.w2_vel);



    sensor_data.left_encoder = (int)((wheel_velocities.w1_vel/rate) + wheel_angle.w_ang1)/encoder_ticks_to_rad;
    sensor_data.right_encoder = (int)((wheel_velocities.w2_vel/rate) + wheel_angle.w_ang2)/encoder_ticks_to_rad;

    ROS_WARN("nusim:left_encoder %d", sensor_data.left_encoder);
    ROS_WARN("nusim:right_encoder %d", sensor_data.right_encoder);

   
    // wheel_angle.w_ang1 = ((wheel_velocities.w1_vel/rate) + wheel_angle.w_ang1);
    // wheel_angle.w_ang2 = ((wheel_velocities.w2_vel/rate) + wheel_angle.w_ang2);
    
    //ROS WARN
    // ROS_WARN("vel_1: %f", wheel_velocities.w1_vel);
    // ROS_WARN("vel_2: %f", wheel_velocities.w2_vel);

    // ROS_WARN("angle_1: %f", wheel_angle.w_ang1);
    // ROS_WARN("angle_2: %f", wheel_angle.w_ang2);

    
    // //ROS_WARN
    // ROS_WARN("Rate: %f", rate);
  
    // ROS_WARN("2vel_1: %f", wheel_velocities.w1_vel);
    // ROS_WARN("2vel_2: %f", wheel_velocities.w2_vel);

    // ROS_WARN("2angle_1: %f", wheel_angle.w_ang1);
    // ROS_WARN("2angle_2: %f", wheel_angle.w_ang2);
    // ROS_WARN("parameter %f", encoder_ticks_to_rad);



    // wheel_angle.w_ang1 = ((wheel_velocities.w1_vel/rate) + wheel_angle.w_ang1);
    // wheel_angle.w_ang2 = ((wheel_velocities.w2_vel/rate) + wheel_angle.w_ang2);

    // current_config = update_config.forward_kinematics(wheel_angle);

    ROS_WARN("x: %f", x);
    ROS_WARN("y: %f", y);
    ROS_WARN("theta: %f", theta);
    ROS_WARN("x_config: %f", current_config.x_config);
    ROS_WARN("y_config: %f", current_config.y_config);
    ROS_WARN("theta_config: %f", current_config.theta_config);


    // x = current_config.x_config;
    // y = current_config.y_config;
    // theta = current_config.theta_config;
    
}



int main(int argc, char ** argv){
    
    
    ros::init(argc, argv, "nusim");
    ros::NodeHandle nh;
    ros::NodeHandle nh2;

    ros::Publisher pub = nh.advertise<std_msgs::UInt64>("timestep_topic", 500);
    // ros::Publisher pub2 = nh2.advertise<sensor_msgs::JointState>("red/joint_states", 100);
    // ros::Publisher pub3 = nh.advertise<nusim::SensorData("red/sensor_data", 1000);


    ros::ServiceServer reset_service = nh.advertiseService("reset", reset_fn);
    // ros::Subscriber sub = nh.subscribe("geometry_msgs/Pose", 1000, pose_callback)
    ros::ServiceServer teleport_service = nh.advertiseService("teleport", teleport_fn);
    sensor_msgs::JointState red_joint_state;

    //marker publisher
    ros::Publisher vis_pub = nh.advertise<visualization_msgs::MarkerArray>("obstacles", 500, true);

    //walls publisher
    ros::Publisher wall_pub = nh.advertise<visualization_msgs::MarkerArray>("walls", 500, true);



    //subsribe to red/wheel_cmd
    ros::Subscriber wheel_cmd_sub = nh.subscribe<nuturtlebot_msgs::WheelCommands>("red/wheel_cmd", 10, wheel_cmd_callback);
    

    //publish to red/sensor_data
    ros::Publisher sensor_pub = nh.advertise<nuturtlebot_msgs::SensorData>("red/sensor_data", 500);

    static tf2_ros::TransformBroadcaster broadcaster;



    //get parameters
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




    // nh.param("x_m", x_m, 1);
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
    ROS_INFO_STREAM("publishing markers");



    ros::Rate r(rate);

    while(ros::ok){

        pub.publish(timestep);
        ros::spinOnce();
        // ROS_INFO("%ld", timestep.data);
        timestep.data++;

        
        wheel_angle.w_ang1 = ((wheel_velocities.w1_vel/rate) + wheel_angle.w_ang1);
        wheel_angle.w_ang2 = ((wheel_velocities.w2_vel/rate) + wheel_angle.w_ang2);

        ROS_WARN("wheel_velocity before fk fn 1: %f", wheel_angle.w_ang1);
        ROS_WARN("wheel_velocity before fk fn 2: %f", wheel_angle.w_ang2);
        current_config = update_config.forward_kinematics(wheel_angle);
        
        ROS_WARN("x_config: %f", current_config.x_config);
        ROS_WARN("y_config: %f", current_config.y_config);
        ROS_WARN("theta_config: %f", current_config.theta_config);


        x = current_config.x_config;
        y = current_config.y_config;
        theta = current_config.theta_config;

        ROS_WARN("x: %f", x);
        ROS_WARN("y: %f", y);
        ROS_WARN("theta: %f", theta);
    


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

