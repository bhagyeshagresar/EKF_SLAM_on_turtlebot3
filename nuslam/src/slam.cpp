/// \file the odometry node broadcasts the transform between odom and the blue-base footprint. The odometry node also 
/// publishes odometry messages on the odom topic
/// subscriber: topics - red/joint_states
/// publisher: topics - /odom



#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/JointState.h>
#include <nav_msgs/Odometry.h>
#include <tf2_ros/transform_broadcaster.h>
#include "nuturtle_control/Set_Pose.h"
#include <std_srvs/Empty.h>
#include "turtlelib/diff_drive.hpp"
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <armadillo>
#include "nuslam/nuslam.hpp"
#include "turtlelib/rigid2d.hpp"
#include "turtlelib/diff_drive.hpp"
#include "visualization_msgs/MarkerArray.h"
#include <tf2_ros/static_transform_broadcaster.h>



static nav_msgs::Odometry odom;
static nav_msgs::Odometry odom2;
static turtlelib::Configuration current_config;
static turtlelib::Wheel_angles wheel_angle;
static turtlelib::Wheels_vel wheel_vel;   
static turtlelib::Twist2D V_twist;
static turtlelib::DiffDrive fwd_diff_drive;
static double init_x_pos{0.0};
static double init_y_pos{0.0};
static double init_theta_pos{0.0};
static std::vector <double> positions;
static std::vector <double> velocities;


//slam variables
static int m{3};
static int n{9};
static slamlib::Estimate2d slam_obj(m, n);
static arma::mat covariance;
static arma::mat prev_state_vector;
static arma::mat state_vector;
static arma::mat q_mat;
static arma::mat r_mat;
static double r{0.0};
static double phi{0.0};
static arma::mat  m_vec(2, 1);
static std::vector <double> x_bar;
static std::vector <double> y_bar;
static double r_noise{100.0};
static double q_noise{1000.0};
static arma::mat z;
static arma::mat map_to_green;
static tf2_ros::StaticTransformBroadcaster static_broadcaster;




void fake_sensor_callback(const visualization_msgs::MarkerArray & msg){
    
    for(int i = 0; i < 3; i++){
        x_bar.at(i) = msg.markers[i].pose.position.x;
        y_bar.at(i) = msg.markers[i].pose.position.y;
    }
   
 

}




void init_fn(double a, double b, double c, double d, double e){
    //slam intialisation steps
    covariance = slam_obj.get_covariance(); // get covariance matrix with size n,n
    state_vector = slam_obj.get_state_vector(); //get state vector with size 1,n
    prev_state_vector = slam_obj.get_prev_state_vector(); //get prev state vector with size 1,n
    q_mat = slam_obj.get_q_matrix(); //get q matrix with 
    r_mat = slam_obj.get_r_matrix(); // get r matrix

    
    
    slam_obj.set_r(r_noise);
    slam_obj.set_q(q_noise);


    q_mat = slam_obj.calculate_q_mat(q_noise, q_mat);
    r_mat = slam_obj.calculate_r_mat(r_noise, r_mat);

    

    covariance(0, 0) = init_theta_pos;
    covariance(1, 1) = init_x_pos;
    covariance(2, 2) = init_y_pos;

    prev_state_vector(0, 0) = init_theta_pos;
    prev_state_vector(1, 0) = init_x_pos;
    prev_state_vector(2, 0) = init_y_pos;

    for(int i = 0; i < m; i++){
        
        r = sqrt(pow(x_bar.at(i), 2) + pow(y_bar.at(i), 2));
        phi = atan2(y_bar.at(i), x_bar.at(i));
        m_vec(0, 0) = (prev_state_vector(1, 0) + r*cos(phi + prev_state_vector(0, 0)));
        m_vec(1, 0) = (prev_state_vector(2, 0) + r*sin(phi + prev_state_vector(0, 0)));
        state_vector = arma::join_cols(state_vector, m_vec);
    }

    
}


arma::mat slam_fn(int m){
    for(int i = 0; i < m; i++){
        //prediction step 1
        state_vector = slam_obj.updated_state_vector(V_twist);

        arma::mat a = slam_obj.calculate_A_matrix(V_twist);
        arma::mat a2 = a.t();
    
        //prediction step 2
        arma::mat sigma = (a*covariance*a2) + q_mat;

        //update step 1
        arma::mat z_hat = slam_obj.calculate_z_hat(i);

        //update step 2
        arma::mat h = slam_obj.calculate_h(m_vec);

        arma::mat ki = sigma*h.t()*(h*sigma*h.t() + r_mat).i();


        //update step 3
        z = slam_obj.calculate_z(x_bar.at(i), y_bar.at(i));
        state_vector = state_vector + ki*(z - z_hat);

        //update step 4
        arma::mat identity(n, n);
        sigma = (identity - (ki*h))*sigma;

        prev_state_vector = state_vector;
        
        return state_vector;


    }
}


/// \brief function to compute the wheel_angles and wheel_velocities from joint_state message
/// \param js_msg - joint state message
void joint_state_callback(const sensor_msgs::JointState::ConstPtr&  js_msg){
    
    positions.resize(2);
    velocities.resize(2);
    positions = js_msg->position;
    velocities = js_msg->velocity;

    wheel_angle.w_ang1 = positions[0]; //wheel angle1
    wheel_angle.w_ang2 = positions[1]; // wheel angle2
    // wheel_angle.w_ang1 = 1.0;
    // wheel_angle.w_ang2 = 2.0;

    // ROS_WARN("wheel angles");

    current_config = fwd_diff_drive.forward_kinematics(wheel_angle);
    


    wheel_vel.w1_vel = velocities[0]; //wheel velocity 1
    wheel_vel.w2_vel = velocities[1]; //wheel velocity 2
    // wheel_vel.w1_vel = 3.0;
    // wheel_vel.w2_vel = 4.0;
    // ROS_WARN("wheel_vel.w1_vel: ", wheel_vel.w1_vel);

}

/// \brief function to set the pose of the blue robot
/// \param req - x, y and theta configuration values of the robot
/// \param res - empty response message 
bool set_pose(nuturtle_control::Set_Pose::Request &req, nuturtle_control::Set_Pose::Response &res){
    

    current_config.x_config = req.x_config;
    current_config.y_config = req.y_config;
    current_config.theta_config = req.theta_config;

    fwd_diff_drive.set_config(current_config);




    return true;

}



int main(int argc, char **argv){

    //initialize rosnode odometry
    ros::init(argc, argv, "slam");

    //create nodehandle object
    ros::NodeHandle nh;


    //subscribe to red/jointStates
    ros::Subscriber js_sub = nh.subscribe("red/joint_states", 10, joint_state_callback);


    //publish on odom topic
    ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 500);
    
    //provide service set_pose
    ros::ServiceServer service = nh.advertiseService("set_pose", set_pose);

    //transform between odom and blue-base_footprint
    tf2_ros::TransformBroadcaster odom_broadcaster;
    tf2_ros::TransformBroadcaster broadcaster_map_to_odom;
    tf2_ros::TransformBroadcaster broadcaster_odom_to_green;


    //subscribe to fake_sensor for SLAM
    ros::Subscriber fake_sub = nh.subscribe("/fake_sensor", 10, fake_sensor_callback);



    ros::Time current_time;
	ros::Time last_time;
	current_time = ros::Time::now();
	last_time = ros::Time::now();

    geometry_msgs::TransformStamped odom_trans;
    geometry_msgs::TransformStamped transformStamped_map_to_odom;
    geometry_msgs::TransformStamped transformStamped_odom_to_green;



    nh.getParam("x0", init_x_pos);
    nh.getParam("y0", init_y_pos);
    nh.getParam("theta0", init_theta_pos);

    
    init_fn(init_x_pos, init_y_pos, init_theta_pos, r_noise, q_noise);

    //map to green-base footprint x, y and theta
    



    ros::Rate r(500);

    
    while(ros::ok()){
        current_time = ros::Time::now();
        

        
        

    
        //calculate twist
        V_twist.x_dot = (fwd_diff_drive.get_radius()*(wheel_vel.w1_vel + wheel_vel.w2_vel))/2;
        V_twist.theta_dot = (fwd_diff_drive.get_radius()*(wheel_vel.w2_vel - wheel_vel.w1_vel))/(2*fwd_diff_drive.get_length_d());


        //get the current configuration of the blue robot
        current_config = fwd_diff_drive.get_config();

        //map - green_base_footprint
        map_to_green = slam_fn(m);
        turtlelib::Transform2D Tmb{turtlelib::Vector2D{map_to_green(1, 0), map_to_green(2, 0)}, map_to_green(0, 0)};
        turtlelib::Transform2D Tob{turtlelib::Vector2D{current_config.x_config, current_config.y_config}, current_config.theta_config};
        turtlelib::Transform2D Tbo = Tob.inv();
        turtlelib::Transform2D Tmo = Tmb*Tbo;

        //map to odom 
        turtlelib::Vector2D v_mo= Tmo.translation();
        double theta_mo = Tmo.rotation();
        


        
        //publish transform between map to odom
        geometry_msgs::TransformStamped transformStamped_map_to_odom;
        transformStamped_map_to_odom.header.stamp = ros::Time::now();
        transformStamped_map_to_odom.header.frame_id = "map";
        transformStamped_map_to_odom.child_frame_id = "odom";
        transformStamped_map_to_odom.transform.translation.x = v_mo.x;
        transformStamped_map_to_odom.transform.translation.y = v_mo.y;
        transformStamped_map_to_odom.transform.translation.z = 0;
        tf2::Quaternion q;
        q.setRPY(0, 0, theta_mo);
        transformStamped_map_to_odom.transform.rotation.x = q.x();
        transformStamped_map_to_odom.transform.rotation.y = q.y();
        transformStamped_map_to_odom.transform.rotation.z = q.z();
        transformStamped_map_to_odom.transform.rotation.w = q.w();
        broadcaster_map_to_odom.sendTransform(transformStamped_map_to_odom);

        
        
    
        
        
        
        //publish transform between odom and green-base_footprint
        geometry_msgs::TransformStamped transformStamped_odom_to_green;
        transformStamped_odom_to_green.header.stamp = ros::Time::now();
        transformStamped_odom_to_green.header.frame_id = "odom";
        transformStamped_odom_to_green.child_frame_id = "green-base_footprint";
        transformStamped_odom_to_green.transform.translation.x = current_config.x_config;
        transformStamped_odom_to_green.transform.translation.y = current_config.y_config;
        transformStamped_odom_to_green.transform.translation.z = 0;
        tf2::Quaternion q2;
        q2.setRPY(0, 0, current_config.theta_config);
        transformStamped_odom_to_green.transform.rotation.x = q2.x();
        transformStamped_odom_to_green.transform.rotation.y = q2.y();
        transformStamped_odom_to_green.transform.rotation.z = q2.z();
        transformStamped_odom_to_green.transform.rotation.w = q2.w();
        broadcaster_odom_to_green.sendTransform(transformStamped_odom_to_green);

      



        //publish transform between odom and blue-base_footprint on tf
        odom_trans.header.stamp = current_time;
        odom_trans.header.frame_id = "world";
        odom_trans.child_frame_id = "blue-base_footprint";

    
        odom_trans.transform.translation.x = current_config.x_config;
        odom_trans.transform.translation.y = current_config.y_config;
        odom_trans.transform.translation.z = 0.0;
        tf2::Quaternion quat;
        quat.setRPY(0, 0, current_config.theta_config);
        odom_trans.transform.rotation.x = quat.x();
        odom_trans.transform.rotation.y = quat.y();
        odom_trans.transform.rotation.z = quat.z();
        odom_trans.transform.rotation.w = quat.w();
        //send the transform
        odom_broadcaster.sendTransform(odom_trans);

       
        //set the header.stamp and header.frame_id for the odometry message
        nav_msgs::Odometry odom;
        odom.header.stamp = current_time;
        odom.header.frame_id = "world";

        //set the position for the robot
        odom.pose.pose.position.x = current_config.x_config;
        odom.pose.pose.position.y = current_config.y_config;
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation.x = quat.x();
        odom.pose.pose.orientation.y = quat.y();
        odom.pose.pose.orientation.z = quat.z();
        odom.pose.pose.orientation.w = quat.w();


        //set the velocity of the robot
        odom.child_frame_id = "blue-base_footprint";
        odom.twist.twist.linear.x = V_twist.x_dot;
        odom.twist.twist.linear.y = 0;
        odom.twist.twist.angular.z = V_twist.theta_dot;
        //publish the odometry message
        odom_pub.publish(odom);



        
        last_time = current_time;
        ros::spinOnce();

        r.sleep();    
    
    
    
    }

}