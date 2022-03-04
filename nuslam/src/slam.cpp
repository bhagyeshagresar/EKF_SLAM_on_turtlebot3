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
#include "nuslam/slamlib.hpp"



static nav_msgs::Odometry odom;
static turtlelib::Configuration current_config;
static turtlelib::Wheel_angles wheel_angle;
static turtlelib::Wheels_vel wheel_vel;   
static turtlelib::Twist2D V_twist;
static turtlelib::DiffDrive fwd_diff_drive;
static double init_x_pos{0.0};
static double init_y_pos{0.0};
static double init_theta_pos{0.0};

//slam variables
static int m{3};
static int n{9};
static slamlib::Estimate2d slam_obj(m, n);
static arma::mat <double> covariance;
static arma::mat <double> prev_state_vector;
static arma::mat <double> state_vector;
static arma::mat <double> q_mat;
static arma::mat <double> r_mat;
static double r{0.0};
static double phi{0.0};
static arma::mat <double> m_vec(2, 1);
static std::vector <double> x_bar;
static std::vector <double> y_bar;
static double r_noise{100.0};
static double q_noise{1000.0};






void fake_sensor_callback(const visualization_msgs::MarkerArray::ConstPtr& msg){
    
    for(int i = 0; i < 3; i++){
        x_bar.at[i] = msg.marker_noise[i].pose.position.x;
        y_bar.at[i] = msg.marker_noise[i].pose.position.y;
    }
    //prediction step 1
    state_vector = Estimate2d::updated_state_vector(V_twist);

    //get A and A_transpose, Covariance
    arma::mat <double> a = Estimate2d::calculate_A_matrix(V_twist);
    arma::mat <double> a2 = a.t();
    
    //prediction step 2
    covariance = (a*covariance*a2) + q_mat;

    //update step 1
    arma::mat z_hat = Estimate2d::calculate_z_hat(m);

    //update step 2
    arma::mat h = Estimate2d::calculate_h(m_vec);

    
    arma::mat ki = covariance*h.t()*(h*covariance*h.t() + r_mat).i());



    //update step 3
    state_vector = state_vector + ki*(z - z_hat)




    


    

}






void initialisation_fn(){
    //slam intialisation steps
    covariance = slam_obj.get_covariance(); // get covariance matrix with size n,n
    state_vector = slam_obj.get_state_vector(); //get state vector with size 1,n
    prev_state_vector = slam_obj.get_prev_state_vector(); //get prev state vector with size 1,n
    q_mat = slam_obj.get_q_matrix(); //get q matrix with 
    r_mat = slam_obj.get_r_matrix(); // get r matrix

    
    covariance(fill::zeros);
    state_vector(fill::zeros);
    prev_state_vector(fill::zeros);
    q_mat(fill::zeros);
    r_mat(fill::zeros);
    
    slam_obj.set_r(r_noise);
    slam_obj.set_q(q_noise);


    q_mat = slam_obj.calculate_q_mat(int q);
    r_mat = slam_obj.calculate_r_mat(int q);

    

    covariance(0, 0) = init_x_pos;
    covariance(1, 1) = init_y_pos;
    covariance(2, 2) = init_theta_pos;

    prev_state_vector(0, 0) = init_x_pos;
    prev_state_vector(0, 1) = init_y_pos;
    prev_state_vector(0, 2) = init_theta_pos;

    for(int i = 0; i < m; i++){
        
        r = sqrt(pow(x_bar, 2) + pow(y_bar, 2));
        phi = atan2(y_bar, x_bar);
        m_vect(0, 0) = (prev_state_vector(0, 0) + r*cos(phi + prev_state_vector(0, 2)));
        m_vect(0, 1) = (prev_state_vector(0, 1) + r*sin(phi + prev_state_vector(0, 2)));
        state_vector = arma::join_cols(state_vector, m_vect);
    }

    
}


void slam_fn(int m){
    for(int i = 0; i < m; i++){
        //prediction step 1
        state_vector = Estimate2d::updated_state_vector(V_twist);

        arma::mat <double> a = Estimate2d::calculate_A_matrix(V_twist);
        arma::mat <double> a2 = a.t();
    
        //prediction step 2
        arma::mat <double> sigma = (a*covariance*a2) + q_mat;

        //update step 1
        arma::mat z_hat = Estimate2d::calculate_z_hat(i);

        //update step 2
        arma::mat h = Estimate2d::calculate_h(m_vec);

        arma::mat ki = sigma*h.t()*(h*sigma*h.t() + r_mat).i();


        //update step 3
        z = calculate_z(x_bar.at(i), y_bar.at(i));
        state_vector = state_vector + ki*(z - z_hat);

        //update step 4
        arma::mat identity(n, n);
        sigma = (identity - (ki*h))*sigma;


    }
}


/// \brief function to compute the wheel_angles and wheel_velocities from joint_state message
/// \param js_msg - joint state message
void joint_state_callback(const sensor_msgs::JointState::ConstPtr&  js_msg){
    
    wheel_angle.w_ang1 = js_msg->position[0]; //wheel angle1
    wheel_angle.w_ang2 = js_msg->position[1]; // wheel angle2


    current_config = fwd_diff_drive.forward_kinematics(wheel_angle);
    


    wheel_vel.w1_vel = js_msg->velocity[0]; //wheel velocity 1
    wheel_vel.w2_vel = js_msg->velocity[1]; //wheel velocity 2

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


    //subscribe to fake_sensor for SLAM
    ros::Subscriber fake_sub = nh.subscribe("/fake_sensor", 10, fake_sensor_callback);



    ros::Time current_time;
	ros::Time last_time;
	current_time = ros::Time::now();
	last_time = ros::Time::now();

    geometry_msgs::TransformStamped odom_trans;


    nh.getParam("x0", init_x_pos);
    nh.getParam("y0", init_y_pos);
    nh.getParam("theta0", init_theta_pos);

    
    initialisation_fn();
   


    ros::Rate r(500);

    
    while(ros::ok()){
        current_time = ros::Time::now();
        


    
        //calculate twist
        V_twist.x_dot = (fwd_diff_drive.get_radius()*(wheel_vel.w1_vel + wheel_vel.w2_vel))/2;
        V_twist.theta_dot = (fwd_diff_drive.get_radius()*(wheel_vel.w2_vel - wheel_vel.w1_vel))/(2*fwd_diff_drive.get_length_d());


        //get the current configuration of the blue robot
        current_config = fwd_diff_drive.get_config();
      
        //publish transform between odom and blue-base_footprint on tf
        odom_trans.header.stamp = current_time;
        odom_trans.header.frame_id = "odom";
        odom_trans.child_frame_id = "green-base_footprint";

    
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
        odom.header.frame_id = "odom";

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