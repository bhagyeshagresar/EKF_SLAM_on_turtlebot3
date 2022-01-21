#include <ros/ros.h>
#include <std_msgs/UInt64.h>
// #include <sensor_msgs/JointState.h>
// #include <tf2/transform_broadcaster.h>
// #include <string>
//service 



int main(int argc, char ** argv){
    
    
    ros::init(argc, argv, "nusim");
    ros::NodeHandle nh("~");
    std_msgs::UInt64 timestep;

    ros::Publisher pub = nh.advertise<std_msgs::UInt64>("timestep_topic", 100);

    ros::Rate r(500);

    while(ros::ok){
        pub.publish(timestep);
        ros::spinOnce();

        r.sleep();
    }
    return 0;
}

