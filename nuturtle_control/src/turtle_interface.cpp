#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include "nuturtlebot_msgs/WheelCommands.h"
#include "nuturtlebot_msgs/SensorData.h"
#include <sensor_msgs/JointState.h>
#include "diff_drive.hpp"
#include "rigid2d.hpp"

static nuturtlebot_msgs::WheelCommands wheel_cmd;
static sensor_msgs::JointState js;
static double motor_cmd_to_rad_sec_1, encoder_ticks_to_rad_1, motor_cmd_to_rad_sec_2, encoder_ticks_to_rad_2;
static double left_encoder_ticks, right_encoder_ticks, left_wheel_angle, right_wheel_angle, left_wheel_velocity, right_wheel_velocity;


void cmd_vel_callback(const geometry_msgs::Twist::ConstPtr& twist_msg){
    Twist2D V;
    
    V.x_dot = twist_msg.linear.x;
    V.theta_dot = twist_msg.angular.z;

    w_vel = inverse_kinematics(V);

    wheel_cmd.left_velocity = w_vel.w1_vel;
    wheel_cmd.right_velocity = w_vel.w2_vel;




}

void sensor_data_callback(const nuturtlebot_msgs::SensorData& sensor_msg){
    left_encoder_ticks  = sensor_msg.left_encoder; // encoder data in ticks
    right_encoder_ticks = sensor_msg.right_encoder;

    //access motor_cmd_to_rad_sec1 and put left_encoder value (rad/sec angular velocity)
    //access motor_cmd_to_rad_sec2 and put right_encoder value

    //encoder ticks to rad (rad) angle

    // js1.position = encoder ticks
    // js1.velocity = motor_cmd_to_rad

    // js2.position = encoder ticks
    // js2.velocity = motor_cmd_to_rad

    left_wheel_angle = (encoder_ticks_to_rad)*left_encoder;
    right_wheel_angle = (encoder_ticks_to_rad)*right_encoder;

    left_wheel_velocity = (motor_cmd_to_rad_sec)*left_encoder;
    right_wheel_velocity = (motor_cmd_to_rad_sec)*right_encoder;

    js.name{left_wheel, right_wheel};
    js.position{left_wheel_angle, right_wheel_angle};
    js.velocity{left_wheel_velocity, right_wheel_velocity};


}







int main(int argc, char **argv){

    ros::init(argc, argv, "turtle_interface");
    ros::NodeHandle nh;



    //subscriber
    ros::Subscriber cmd_vel_sub = nh.subscribe("cmd_vel", 1000, cmd_vel_callback);
    ros::Subscriber sensor_data_sub = nh.subscribe("sensor_data", 1000, sensor_data_callback)
 
    //publisher
    ros::Publisher wheel_cmd_pub = nh.advertise<nuturtlebot_msgs::WheelCommands>("wheel_cmd", 100);
    ros::Publisher joint_state_pub = nh.advertise<sensor_msgs::JointState>("joint_states", 100);

    nh.getParam("motor_cmd_to_radsec", motor_cmd_to_rad_sec);

    nh.getParam("encoder_ticks_to_rad", encoder_ticks_to_rad);





    //Twist forward_kinematics

    while(ros::ok()){
        ros::spinOnce();


        wheel_cmd_pub.publish(wheel_cmd);
        joint_state_pub.publish(joint_state_pub);





    }
    



}
