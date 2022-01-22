#include <ros/ros.h>
#include <std_msgs/UInt64.h>
#include <ros/console.h>
#include <iostream>
#include "nusim/Reset.h"

// #include <sensor_msgs/JointState.h>
// #include <tf2/transform_broadcaster.h>
// #include <string>
//service 

static std_msgs::UInt64 timestep;


std_msgs::UInt64 reset_fn(nusim::Reset::Request &res){
    timestep.data = 0;
    return timestep;


}




int main(int argc, char ** argv){
    
    
    ros::init(argc, argv, "nusim");
    ros::NodeHandle nh("~");
    // std_msgs::UInt64 timestep {0};

    ros::Publisher pub = nh.advertise<std_msgs::UInt64>("timestep_topic", 100);
    ros::ServiceServer service = n.advertiseService("reset", reset_fn);
    // std_msgs::UInt64 timestep;
    timestep.data = 0;



    ros::Rate r(500);

    while(ros::ok){

        pub.publish(timestep);
        ros::spinOnce();
        ROS_INFO("%d", timestep.data);
        // std::cout << "timestep " << timestep << endl;
        timestep.data++;
        r.sleep();
    }
    return 0;
}

