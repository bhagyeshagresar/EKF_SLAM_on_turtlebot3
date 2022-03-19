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
#include "visualization_msgs/Marker.h"
#include <tf2_ros/static_transform_broadcaster.h>



static nav_msgs::Odometry odom;
static turtlelib::Configuration current_config;
static turtlelib::Wheel_angles wheel_angle;
static turtlelib::Wheels_vel wheel_vel;   
static turtlelib::Twist2D V_twist;
static turtlelib::DiffDrive fwd_diff_drive;
static turtlelib::DiffDrive fwd_diff_drive_slam;

static double init_x_pos{0.0};
static double init_y_pos{0.0};
static double init_theta_pos{0.0};
static std::vector <double> positions;
static std::vector <double> velocities;



static double radius{0.038};
static double r_noise{100};
static double q_noise{1};
static std::vector <double> x_bar;
static std::vector <double> y_bar;
static int state = 1;
static arma::mat map_to_green(9, 1, arma::fill::zeros);
static arma::mat delta_z(2, 1, arma::fill::zeros);

static arma::mat temp_vec(6, 1, arma::fill::zeros);
static double r_j{0.0};
static double phi{0.0};
static slamlib::Estimate2d slam_obj(r_noise, q_noise);
// static arma::mat sigma_prev = slam_obj.get_covariance();

// static arma::mat predict_vector(9, 1, arma::fill::zeros);
// static arma::mat init_vector(9, 1, arma::fill::zeros);




arma::mat slam_fn(arma::mat temp_vec){
    turtlelib::Configuration current_config_slam = fwd_diff_drive_slam.get_config();
    
   
    //predict step 1
    arma::mat predict_vector = slam_obj.get_predict_vector();
    predict_vector(0, 0) = turtlelib::normalize_angle(current_config_slam.theta_config);
    predict_vector(1, 0) = current_config_slam.x_config;
    predict_vector(2, 0) = current_config_slam.y_config;
    predict_vector.print("step 1 predict vector");
    slam_obj.set_predict_vector(predict_vector);


    //predict step 2
    arma::mat a = slam_obj.calculate_A_matrix(V_twist);
    arma::mat a2 = a.t();
    a.print("step 2: a matrix");
    
    //predict step 3
    arma::mat q_mat = slam_obj.get_q_matrix();
    arma::mat sigma_prev = slam_obj.get_covariance();
    sigma_prev.print("sigma_prev");
    arma::mat sigma_new = (a*sigma_prev*a2) + q_mat;
    sigma_new.print("step 3: sigma");

    for(int i = 0; i < 3; i++){
        predict_vector.print("predict vector before z_hat");
        arma::mat z_hat = slam_obj.calculate_z_hat(i);
        z_hat.print("step 4: z_hat");
        predict_vector.print("predict vector after z_hat");


        // update step 2
        arma::mat h = slam_obj.calculate_h(i);
        h.print("step 5: h");

        arma::mat r_matrix = slam_obj.get_r_matrix();
        arma::mat h_tranpose = h.t();
        h_tranpose.print("step 6: h_transpose");
        r_matrix.print("step 7: r_matrix");
        arma::mat mat_inv = arma::inv((h*sigma_new*h_tranpose) + r_matrix);
        mat_inv.print("step 8: matInv");
        arma::mat ki = (sigma_new * h_tranpose * mat_inv);
        ki.print("step 9: ki");

        //update step 3
        double g = temp_vec((2*i), 0);
        double k = temp_vec((2*i)+1, 0);
        arma::mat z = slam_obj.calculate_z(g, k);
        z.print("step 10: z");

        arma::mat delta_z(2, 1, arma::fill::zeros);
        delta_z(0, 0) = z(0, 0) - z_hat(0, 0);
        delta_z(1, 0) = z(1, 0) - z_hat(1, 0);
        delta_z(1, 0) = turtlelib::normalize_angle(delta_z(1,0));
        delta_z.print("step 11: delta_z");
        arma::mat temp_mat = (ki*(delta_z));
        temp_mat.print("temp mat");
        // temp_mat(3, 0) = 0.0;
        // temp_mat(4, 0) = 0.0;
        predict_vector.print("step 12: state_vector_1");
        predict_vector = predict_vector + temp_mat;
        predict_vector.print("step 13: state_vector_1");
        


        // update step 4
        arma::mat identity = arma::eye(9, 9);
        sigma_new = (identity - (ki*h))*sigma_new;
        sigma_new.print("step 14: sigma");

        // abort();






    }


    // predict_vector = state_vector_1;
    sigma_prev = sigma_new;
    slam_obj.set_covariance(sigma_prev);
    slam_obj.set_predict_vector(predict_vector);

    





    return predict_vector;
}










void fake_sensor_callback(const visualization_msgs::MarkerArray & msg){
    x_bar.resize(3);
    y_bar.resize(3);
    // ROS_WARN("m: %d", m);
    for(int i = 0; i < 3; i++){
        // ROS_WARN("m: %d", m);
        x_bar.at(i) = msg.markers[i].pose.position.x;
        y_bar.at(i) = msg.markers[i].pose.position.y;
        // ROS_WARN("x_bar.at(i) %f", x_bar.at(i));
       
        r_j = std::sqrt(std::pow(x_bar.at(i), 2) + std::pow(y_bar.at(i), 2));
        phi = std::atan2(y_bar.at(i), x_bar.at(i));
        ROS_WARN("r_j: %f", r_j);
        ROS_WARN("phi: %f", phi);


        temp_vec(2*i, 0) = r_j;
        temp_vec((2*i)+1, 0) = phi;

    }
    
    // prev_state_vector.print("prev vector after init");

    if(state == 1){
        //vector initialised with landmarks
        slam_obj.init_fn(temp_vec);

    }
    state = 0;
    
   
    map_to_green = slam_fn(temp_vec);
   
    // auto fake_sensor_time = ros::Time::now();
    // ROS_WARN_STREAM("fake sensor time: " << fake_sensor_time);
    // map_to_green.print("map");
    // ROS_WARN("state changed");
   
 

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
   

    current_config = fwd_diff_drive.forward_kinematics(wheel_angle);
    // current_config_slam = fwd_diff_drive_slam.forward_kinematics(wheel_angle);

    fwd_diff_drive_slam.forward_kinematics(wheel_angle);

    wheel_vel.w1_vel = velocities[0]; //wheel velocity 1
    wheel_vel.w2_vel = velocities[1]; //wheel velocity 2
    
    V_twist.x_dot = (((fwd_diff_drive.get_radius()*(wheel_vel.w1_vel +  wheel_vel.w2_vel))/2))/(500.0);
    V_twist.theta_dot = (((fwd_diff_drive.get_radius()*(wheel_vel.w2_vel -  wheel_vel.w1_vel))/(2*fwd_diff_drive.get_length_d())))/(500.0);
    // V_twist.theta_dot = 0.0;
    // V_twist.y_dot = 0.0;
    // V_twist.x_dot = 0.0;
    // V_twist.theta_dot = 0.1;
    // V_twist.y_dot = 0.0;

    // ROS_WARN("V_twist is: %f", V_twist.x_dot);
    // ROS_WARN("V_twist is: %f", V_twist.theta_dot);

    // auto joint_state_time = ros::Time::now();
    // ROS_WARN_STREAM("joint state time: " << joint_state_time);



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
    ros::NodeHandle n;


    //subscribe to red/jointStates
    ros::Subscriber js_sub = n.subscribe("red/joint_states", 10, joint_state_callback);


    //publish on odom topic
    ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom", 500);
    
    //provide service set_pose
    ros::ServiceServer service = n.advertiseService("set_pose", set_pose);

    //transform between odom and blue-base_footprint
    tf2_ros::TransformBroadcaster odom_broadcaster;
  
    n.getParam("x0", init_x_pos);
    n.getParam("y0", init_y_pos);
    n.getParam("theta0", init_theta_pos);

    //subscribe to fake_sensor for SLAM
    ros::Subscriber fake_sub = n.subscribe("/fake_sensor", 10, fake_sensor_callback);

    ros::Publisher slam_marker_pub = n.advertise<visualization_msgs::MarkerArray>("/slam_markers", 500, true);

    // state = 1;

    ros::Time current_time;
	ros::Time last_time;
	current_time = ros::Time::now();
	last_time = ros::Time::now();

    tf2_ros::TransformBroadcaster broadcaster_map_to_odom;
    tf2_ros::TransformBroadcaster broadcaster_odom_to_green;


    

   
    //map to green-base footprint x, y and theta
    



    ros::Rate r(500);

  
    while(ros::ok()){

        current_time = ros::Time::now();

    
        //calculate twist
       

        //get the current configuration of the blue robot
        current_config = fwd_diff_drive.get_config();


        //map - green_base_footprint
        // arma::mat map_to_green = slam_fn(m);
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
        transformStamped_odom_to_green.transform.translation.y =  current_config.y_config;
        transformStamped_odom_to_green.transform.translation.z = 0;
        tf2::Quaternion q2;
        q2.setRPY(0, 0,  current_config.theta_config);
        transformStamped_odom_to_green.transform.rotation.x = q2.x();
        transformStamped_odom_to_green.transform.rotation.y = q2.y();
        transformStamped_odom_to_green.transform.rotation.z = q2.z();
        transformStamped_odom_to_green.transform.rotation.w = q2.w();
        broadcaster_odom_to_green.sendTransform(transformStamped_odom_to_green);

      



        //publish transform between odom and blue-base_footprint on tf
        geometry_msgs::TransformStamped odom_trans;
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

        visualization_msgs::MarkerArray slam_array;
        //publish slam markers
        for (int i = 0; i < 3; i++){
            // map_to_green.print("map");
            

            visualization_msgs::Marker slam_marker;
            slam_marker.header.frame_id = "map";
            slam_marker.header.stamp = ros::Time::now();
            slam_marker.ns = "slam_marker";
            slam_marker.id = i;
            slam_marker.type = visualization_msgs::Marker:: CYLINDER;
            slam_marker.action = visualization_msgs::Marker::ADD;
            slam_marker.pose.position.x = map_to_green(3+(2*i), 0);
            slam_marker.pose.position.y = map_to_green(4+(2*i), 0);
            slam_marker.pose.position.z = 0;
            slam_marker.pose.orientation.x = 0.0;
            slam_marker.pose.orientation.y = 0.0;
            slam_marker.pose.orientation.z = 0.0;
            slam_marker.pose.orientation.w = 1.0;
            slam_marker.scale.x = radius*2;
            slam_marker.scale.y = radius*2;
            slam_marker.scale.z = 0.25;
            slam_marker.color.a = 1.0;
            slam_marker.color.r = 0.0;
            slam_marker.color.g = 1.0;
            slam_marker.color.b = 0.0;
            slam_marker.lifetime = ros::Duration();
            
            slam_array.markers.push_back(slam_marker);
            
            }
        slam_marker_pub.publish(slam_array);



        
        last_time = current_time;
        ros::spinOnce();

        r.sleep();    
    
    
    
    }

}