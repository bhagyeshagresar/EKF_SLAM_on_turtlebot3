#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <nav_msgs/Odometry.h>
#include <tf2_ros/transform_broadcaster.h>


nav_msgs::Odometry odom;

void joint_state_callback(const sensor_msgs::JointState::ConstPtr&  js_msg){
    odom.pose = js_msg.position;
    odom.twist = js_msg.velocity;

}





int main(int argc, char **argv){
    
    ros::init(argc, argv, "odometry");
    ros::NodeHandle nh;


    //subscribe to jointStates
    ros::Subscriber js_sub = nh.subscribe("joint_states", 1000, joint_state_callback);


    //publish on odom topic
    ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 100);
    
    
    
    tf::TransformBroadcaster odom_broadcaster;





    // initial position of robot at origin of odom coordinate frame
	double x = 0.0; 
	double y = 0.0;
	double th = 0;



    // velocity - base_link moves in the odom_frame 
	double vx = 0.1;
	double vy = -0.1;
	double vth = 0.1;


    ros::Time current_time;
	ros::Time last_time;
	current_time = ros::Time::now();
	last_time = ros::Time::now();

    ros::Rate r(1);

    
    
    while(ros::ok()){
        ros::spinOnce();
        current_time = ros::Time::now();
        
        double dt = 0.01;
        double delta_x = (vx * cos(th) - vy * sin(th)) * dt;
        double delta_y = (vx * sin(th) + vy * cos(th)) * dt;
        double delta_th = vth * dt;
   
        x += delta_x;
        y += delta_y;
        th += delta_th;

        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(th);
      
        //publish transform over tf
        geometry_msgs::TransformStamped odom_trans;
        odom_trans.header.stamp = current_time;
        odom_trans.header.frame_id = "odom_id";
        odom_trans.child_frame_id = "body_id";

    
        odom_trans.transform.translation.x = x;
        odom_trans.transform.translation.y = y;
        odom_trans.transform.translation.z = 0.0;
        odom_trans.transform.rotation = odom_quat;

        //send the transform
        odom_broadcaster.sendTransform(odom_trans);

        //publish odometry message over ros
        nav_msgs::Odometry odom;
        odom.header.stamp = current_time;
        odom.header.frame_id = "odom_id";
        odom.header.child_frame_id = "body_id";

        //set the position
        odom.pose.pose.position.x = x;
        odom.pose.pose.position.y = y;
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation = odom_quat;

        //set the velocity
        odom.child_frame_id = "base_link";
        odom.twist.twist.linear.x = vx;
        odom.twist.twist.linear.y = vy;
        odom.twist.twist.angular.z = vth;

        odom_pub.publish(odom);

        last_time = current_time;
        r.sleep();    
    
    
    
    }

}