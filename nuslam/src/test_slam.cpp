// #include <ros/ros.h>
// #include <ros/console.h>
// #include <sensor_msgs/JointState.h>
// #include <nav_msgs/Odometry.h>
// #include <tf2_ros/transform_broadcaster.h>
// #include "nuturtle_control/Set_Pose.h"
// #include <std_srvs/Empty.h>
// #include "turtlelib/diff_drive.hpp"
// #include <geometry_msgs/TransformStamped.h>
// #include <tf2/LinearMath/Quaternion.h>
// #include <tf2_ros/static_transform_broadcaster.h>
// #include <armadillo>
// #include "nuslam/nuslam.hpp"
// #include "turtlelib/rigid2d.hpp"
// #include "turtlelib/diff_drive.hpp"
// #include "visualization_msgs/MarkerArray.h"
// #include "visualization_msgs/Marker.h"
// #include <tf2_ros/static_transform_broadcaster.h>



// static nav_msgs::Odometry odom;
// static turtlelib::Configuration current_config;
// static turtlelib::Wheel_angles wheel_angle;
// static turtlelib::Wheels_vel wheel_vel;   
// static turtlelib::Twist2D V_twist;
// static turtlelib::DiffDrive fwd_diff_drive;
// static double init_x_pos{0.0};
// static double init_y_pos{0.0};
// static double init_theta_pos{0.0};
// static std::vector <double> positions;
// static std::vector <double> velocities;


// //slam variables
// static int m{1};
// static int n{9};
// static double radius{0.038};
// static double r_noise{0.1};
// static double q_noise{1.0};
// // static double r{0.0};
// // static double phi{0.0};
// static std::vector <double> x_bar;
// static std::vector <double> y_bar;
// static int state = 1;
// // static int flag = 7;
// static arma::mat map_to_green(n, 1, arma::fill::zeros);
// static arma::mat delta_z(2, 1, arma::fill::zeros);
// // static arma::mat state_vector_1(n, 1);
// static slamlib::Estimate2d test_obj(m, n, r_noise, q_noise);

        




// int main(){
//     test_obj.init_fn();

//     arma::mat state_vector_test = test_obj.updated_state_vector(V_twist);
//     state_vector_test.print("step 2: state_vector_test");

//     arma::mat a = test_obj.calculate_A_matrix(V_twist, n);
//     arma::mat a2 = a.t();
//     a.print("step 4: a");
//     a.t().print("step 4: a+t");

//     arma::mat sigma = test_obj.get_covariance();
//     arma::mat q_mat = test_obj.get_q_matrix();
//     sigma = (a*sigma*a2) + q_mat;
//     sigma.print("step 6: sigma");

//     // // //update step 1
//     arma::mat z_hat = test_obj.calculate_z_hat(0);
//     z_hat.print("step 8: z_hat");

//     // //update step 2
//     arma::mat h = test_obj.calculate_h(0);
//     h.print("step 10: h");

//     arma::mat r_matrix = test_obj.get_r_matrix();
//     arma::mat h_tranpose = h.t();
//     h_tranpose.print("step 11: h_transpose");
//     r_matrix.print("step 12: r_matrix");
//     arma::mat mat_inv = arma::inv(h * sigma * h_tranpose + r_matrix);
//     mat_inv.print("step 13: matInv");
//     arma::mat ki = (sigma * h_tranpose * mat_inv);
//     ki.print("step 12: ki");


//     //update step 3
//     test_obj.calculate_range_bearing(x_bar.at(0), y_bar.at(0));
//     arma::mat z = test_obj.calculate_z();
//     z.print("step 14: z");

//     // // state_vector.print("step 15: state_vector");
//     arma::mat delta_z(2, 1, arma::fill::zeros);
//     delta_z(0, 0) = z(0, 0) - z_hat(0, 0);
//     delta_z(1, 0) = z(1, 0) - z_hat(1, 0);
//     delta_z(1, 0) = turtlelib::normalize_angle(delta_z(1,0));
//     delta_z.print("step 15: delta_z");
//     arma::mat k_delta = (ki*(delta_z));
//     k_delta(3, 0) = 0.0;
//     k_delta(4, 0) = 0.0;
//     state_vector_test = state_vector_test + k_delta;

//     k_delta.print("step 15: ki*delta_z");
//     state_vector_test.print("step 16: state_vector_1");

    
//     // state_vector_1.print("step 16: state_vector");


//     //update step 4
//     arma::mat identity = arma::eye(n, n);
//     sigma = (identity - (ki*h))*sigma;
//     sigma.print("step 18: sigma");

//     // prev_state_vector.print("step 19: prev_state_vector");
//     arma::mat prev_vector = test_obj.get_prev_state_vector();
//     prev_vector = state_vector_test;
//     prev_vector.print("step 20: prev_state_vector");
//     double theta = state_vector_test(0, 0);
//     state_vector_test(0, 0) = turtlelib::normalize_angle(theta);

    


//     return 0;
// }

