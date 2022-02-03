#include <ros/rosh.h>
#include <geometry_msgs/Twist.h>
#include "nuturtlebot_msgs/WheelCommands.h"
#include "nuturtlebot_msgs/SensorData.h"
#include <sensor_msgs/JointState.h>










int main(int argc, char **argv){

    ros::init(argc, argv, "turtle_interface");
    ros::NodeHandle nh;



    //subscriber
    ros::Subscriber sub = nh.subscribe("cmd_vel", 1000, cmd_vel_callback);

    //publisher
    ros::Publisher pub = nh.advertise<nuturtlebot_msgs::WheelCommands>("wheel_cmd", 100);


    



}
