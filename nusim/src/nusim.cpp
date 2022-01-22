#include <ros/ros.h>
#include <std_msgs/UInt64.h>
#include <ros/console.h>
#include <iostream>
#include <std_srvs/Empty.h>
#include <sensor_msgs/JointState.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>


static std_msgs::UInt64 timestep;
static double x, y, theta, rate;


bool reset_fn(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
    timestep.data = 0;
    return true;
}






int main(int argc, char ** argv){
    
    
    ros::init(argc, argv, "nusim");
    ros::NodeHandle nh("~");
    ros::NodeHandle nh2;

    ros::Publisher pub = nh.advertise<std_msgs::UInt64>("timestep_topic", 100);
    ros::Publisher pub2 = nh2.advertise<sensor_msgs::JointState>("red/joint_states", 100);

    ros::ServiceServer service = nh.advertiseService("reset", reset_fn);
    sensor_msgs::JointState red_joint_state;


    //get parameters
    nh.param("x", x, 0.0);
    nh.param("y", y, 0.0);
    nh.param("theta", theta, 0.0);
    nh.param("rate", rate, 500.0);


    timestep.data = 0;
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
    static_transformStamped.child_frame_id = "red_base_footprint";
    static_transformStamped.transform.translation.x = x;
    static_transformStamped.transform.translation.y = y;
    static_transformStamped.transform.translation.z = 0;
    tf2::Quaternion q;
    q.setRPY(0, 0, theta);
    static_transformStamped.transform.rotation.x = q.x();
    static_transformStamped.transform.rotation.y = q.y();
    static_transformStamped.transform.rotation.z = q.z();
    static_transformStamped.transform.rotation.w = 1;




    ros::Rate r(rate);

    while(ros::ok){

        pub.publish(timestep);
        ros::spinOnce();
        ROS_INFO("%ld", timestep.data);
        // std::cout << "timestep " << timestep << endl;
        timestep.data++;
        r.sleep();
    }
    return 0;
}

