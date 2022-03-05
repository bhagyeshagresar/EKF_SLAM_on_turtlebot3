/// \file
/// \brief nusim node subscribes to wheel_cmd topic and published sensor_data message. The nusim node 
/// provides a simulation for the robot and other objects in the environment. The node provides a reset service
/// and teleport service to reset the simulation to initial state and to teleport the turtlebot to a specific location
/// subscriber topic: red/wheel_cmd
/// publisher topics: red/sensor_data, /obstacles, /walls


#include <ros/ros.h> 
#include <std_msgs/UInt64.h>
#include <ros/console.h>
#include <iostream>
#include <std_srvs/Empty.h>
#include <sensor_msgs/JointState.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include "nusim/Teleport.h"
#include "visualization_msgs/Marker.h"
#include "visualization_msgs/MarkerArray.h"
#include <vector>
#include "nuturtlebot_msgs/WheelCommands.h"
#include "nuturtlebot_msgs/SensorData.h"
#include "turtlelib/diff_drive.hpp"
#include <nav_msgs/Path.h>
#include <sensor_msgs/LaserScan.h>
#include <random>
#include <math.h>
#include "turtlelib/diff_drive.hpp"
#include "turtlelib/rigid2d.hpp"


/// Define the variables
static std_msgs::UInt64 timestep;
static double x, y, theta, rate, radius;
static int num_markers;
static std::vector <double> x_m;
static std::vector <double> y_m;
static std::vector <double> wall_xpos{3.0, -3.0, 0.0, 0.0};
static std::vector <double> wall_ypos{0.0, 0.0, 3.0, -3.0};
static std::vector <double> x_length;
static std::vector <double> y_length;


static double encoder_ticks_to_rad{0.0}, motor_cmd_to_radsec{0.0};
static int wheel_velocity_left {0};
static int wheel_velocity_right {0};
static turtlelib::DiffDrive update_config;
static turtlelib::Wheels_vel wheel_velocities;
static turtlelib::Wheel_angles wheel_angle;
static turtlelib::Configuration current_config;
static double left_wheel_noise{0.0};
static double right_wheel_noise{0.0};
static double old_x{0.0};
static double old_y{0.0};
static double old_theta{0.0};

//define relative position variables
static turtlelib::Vector2D V_rel;

static double obstacle_noise{0.0};
static double distance_1 {0.0};
static double distance_2 {0.0};
static double distance_3 {0.0};


// define variables for intersection 
static double x_r{0.0};
static double x_max{0.0};
static double y_r{0.0};
static double y_max{0.0};
static double max_range{3.5};
static double theta_range{0.0};
static double x_p{0.0};
static double y_p{0.0};
static double x_pmax{0.0};
static double y_pmax{0.0};
static double d_x{0.0};
static double d_y{0.0};
static double d_r{0.0};
static double delta{0.0};
static double x_int_pos{0.0};
static double y_int_pos{0.0};
static double x_int_neg{0.0};
static double y_int_neg{0.0};
static double sgn_dy{0.0};
static double mod_dy{0.0};
static double discriminant{0.0};
static std::vector <double> x_int_array;
static std::vector <double> y_int_array;
static turtlelib::Vector2D V_rint_pos;
static turtlelib::Vector2D V_rint_neg;
static double d_xint_pos{0.0};
static double d_yint_pos{0.0};
static double d_xint_neg{0.0};
static double d_yint_neg{0.0};
static double intersection_distance_pos{0.0};
static double intersection_distance_neg{0.0};
static int num_readings = 360;
// staticstd::vector <double> a;
// static std::vector <double> a[num_readings];
// a.resize(num_readings);
static double d_new{0.0};
static double x_1{0.0};
static double y_1{0.0};
static double x_2{0.0};
static double y_2{0.0};
static double x_3{0.0};
static double y_3{0.0};
static double x_4{0.0};
static double y_4{0.0};
static double intersection_distance_wall{0.0};
static double p_x{0.0};
static double p_y{0.0};
static double d_x_new{0.0};
static double d_y_new{0.0};




std::mt19937 & get_random()
 {
     // static variables inside a function are created once and persist for the remainder of the program
     static std::random_device rd{}; 
     static std::mt19937 mt{rd()};
     // we return a reference to the pseudo-random number genrator object. This is always the
     // same object every time get_random is called
     return mt;
 }


//calculate sgn(x)
int sgn_fn(double a){
    if (a < 0.0){
        return -1;
    }
    return 1;
}


//calculate mod(x)
double mod_fn(double a){
    if(a < 0.0){
        return a;
    }
    return a;

}

//calculate distance
double distance_fn(double a, double b){
    return sqrt(pow(a, 2) + pow(b, 2));
}



//defin sensor_data_message
static nuturtlebot_msgs::SensorData sensor_data;

//define path
static nav_msgs::Path path1; 
static nav_msgs::Path path2; 
static nav_msgs::Path path3; 



/// \brief function to reset to the initial state of the simulation
/// \param req - empty request message
/// \param res - empty response message 
bool reset_fn(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res){
    timestep.data = 0;
    x = 0;
    y = 0;
    return true;
}



/// \brief function to teleport the robot to a specific location
/// \param req - the request message takes float64 x, y and theta
/// \param res - the response message is an empty response
bool teleport_fn(nusim::Teleport::Request &req, nusim::Teleport::Response &res){
    x = req.x;
    y = req.y;
    theta = req.theta;
    return true;

}

/// \brief nusim node subscribes to red/wheel_cmd topic. The callback function set wheel velocities of the robot
/// and also calculates encoder ticks for left and right wheels
/// \param msg - nuturtlebot_msgs/WheelCommands 
void wheel_cmd_callback(const nuturtlebot_msgs::WheelCommands::ConstPtr& msg){

   

    wheel_velocity_left = msg->left_velocity; //motor cmd to rad/sec
    wheel_velocity_right = msg->right_velocity;

    
    
    std::normal_distribution<> d1(wheel_velocity_left, 0.01);
    std::normal_distribution<> d2(wheel_velocity_right, 0.01);

    //wheel velocities with noise 
    left_wheel_noise = d1(get_random());
    right_wheel_noise = d2(get_random());


    wheel_velocities.w1_vel = (left_wheel_noise*0.024);
    wheel_velocities.w2_vel = (right_wheel_noise*0.024);

   

    sensor_data.left_encoder = (int)((wheel_velocities.w1_vel/rate) + wheel_angle.w_ang1)/encoder_ticks_to_rad;
    sensor_data.right_encoder = (int)((wheel_velocities.w2_vel/rate) + wheel_angle.w_ang2)/encoder_ticks_to_rad;

    // ROS_WARN("sensor_data left_encoder %d", sensor_data.left_encoder);
    // ROS_WARN("sensor_data right_encoder %d", sensor_data.right_encoder);
   
    
}



int main(int argc, char ** argv){
    
    //initialise rosnode nusim
    ros::init(argc, argv, "nusim");

    //create nodehandle objects
    ros::NodeHandle nh;
    ros::NodeHandle nh2;

    //publish to timestep topic
    ros::Publisher pub = nh.advertise<std_msgs::UInt64>("timestep_topic", 500);
   
    //advertise the reset service
    ros::ServiceServer reset_service = nh.advertiseService("reset", reset_fn);
    
    //advertise the teleport service
    ros::ServiceServer teleport_service = nh.advertiseService("teleport", teleport_fn);
    sensor_msgs::JointState red_joint_state;

    //publisher for cylindrical obstacles
    ros::Publisher vis_pub = nh.advertise<visualization_msgs::MarkerArray>("obstacles", 500, true);

    //walls publisher
    ros::Publisher wall_pub = nh.advertise<visualization_msgs::MarkerArray>("walls", 500, true);



    //subsribe to red/wheel_cmd
    ros::Subscriber wheel_cmd_sub = nh.subscribe<nuturtlebot_msgs::WheelCommands>("red/wheel_cmd", 10, wheel_cmd_callback);
    

    //publish to red/sensor_data
    ros::Publisher sensor_pub = nh.advertise<nuturtlebot_msgs::SensorData>("red/sensor_data", 500);

    //publish path
    ros::Publisher path_pub1 = nh.advertise<nav_msgs::Path>("red_path", 500);
    ros::Publisher path_pub2 = nh.advertise<nav_msgs::Path>("blue_path", 500);
    ros::Publisher path_pub3 = nh.advertise<nav_msgs::Path>("green_path", 500);


    //publish laser_scan
    ros::Publisher laser_pub = nh.advertise<sensor_msgs::LaserScan>("laser_scan", 500, true);

    //publish markerarray on te fake sensor topic
    ros::Publisher fake_pub = nh.advertise<visualization_msgs::MarkerArray>("/fake_sensor", 500, true);


    static tf2_ros::TransformBroadcaster broadcaster;

    double laser_frequency = 5.0;
    // double ranges[num_readings];
    // double intensities[num_readings];




    //get parameters from basic_world.yaml
    nh.getParam("x0", x);
    nh.getParam("y0", y);
    nh.param("theta0", theta);
    nh.param("rate", rate, 500.0);
    nh.getParam("num_markers", num_markers);
    nh.getParam("x_m", x_m);
    nh.getParam("y_m", y_m);
    nh.getParam("radius", radius);
    nh.getParam("encoder_ticks_to_rad", encoder_ticks_to_rad);
    nh.getParam("motor_cmd_to_rad_sec", motor_cmd_to_radsec);
    nh.getParam("x_length", x_length);
    nh.getParam("y_length", y_length);



    //visualize the walls
    visualization_msgs::MarkerArray wall_array;
        for(int j = 0; j < 4; j++){
            visualization_msgs::Marker wall_marker;
            wall_marker.header.frame_id = "world";
            wall_marker.header.stamp = ros::Time::now();
            wall_marker.id = j;
            wall_marker.type = visualization_msgs::Marker::CUBE;
            wall_marker.action = visualization_msgs::Marker::ADD;
            wall_marker.pose.position.x = wall_xpos.at(j);
            wall_marker.pose.position.y = wall_ypos.at(j);
            wall_marker.pose.position.z = 0;
            wall_marker.pose.orientation.x = 0.0;
            wall_marker.pose.orientation.y = 0.0;
            wall_marker.pose.orientation.z = 0.0;
            wall_marker.pose.orientation.w = 1.0;
            wall_marker.scale.x = x_length.at(j);
            wall_marker.scale.y = y_length.at(j);
            wall_marker.scale.z = 0.25;
            wall_marker.color.a = 1.0;
            wall_marker.color.r = 1.0;
            wall_marker.color.g = 0.0;
            wall_marker.color.b = 0.0;
            wall_marker.lifetime = ros::Duration();
            
            wall_array.markers.push_back(wall_marker);




        }
    wall_pub.publish(wall_array);


        


    //visualize the cylindrical obstacles
    timestep.data = 0;
    visualization_msgs::MarkerArray marker_array;
        for (int i = 0; i < num_markers; i++){
            visualization_msgs::Marker marker;
            marker.header.frame_id = "world";
            marker.header.stamp = ros::Time::now();
            marker.ns = "obstacles";
            marker.id = i;
            marker.type = visualization_msgs::Marker:: CYLINDER;
            marker.action = visualization_msgs::Marker::ADD;
            marker.pose.position.x = x_m.at(i);
            marker.pose.position.y = y_m.at(i);
            marker.pose.position.z = 0;
            marker.pose.orientation.x = 0.0;
            marker.pose.orientation.y = 0.0;
            marker.pose.orientation.z = 0.0;
            marker.pose.orientation.w = 1.0;
            marker.scale.x = radius*2;
            marker.scale.y = radius*2;
            marker.scale.z = 0.25;
            marker.color.a = 1.0;
            marker.color.r = 1.0;
            marker.color.g = 0.0;
            marker.color.b = 0.0;
            ROS_INFO("check for rviz");
            marker.lifetime = ros::Duration();
            
            marker_array.markers.push_back(marker);




        }
    vis_pub.publish(marker_array);
    ROS_INFO_STREAM("publishing markers");


    
    
    std::normal_distribution<> m_n(0, 0.01);




    ros::Rate r(rate);

    while(ros::ok()){

        pub.publish(timestep);
        timestep.data++;
        
        // old_x = current_config.x_config;
        // old_y = current_config.y_config;
        // old_theta = current_config.theta_config;
        


        //calculate the wheel angles using wheel velocities
        wheel_angle.w_ang1 = ((wheel_velocities.w1_vel/rate) + wheel_angle.w_ang1);
        wheel_angle.w_ang2 = ((wheel_velocities.w2_vel/rate) + wheel_angle.w_ang2);

        // wheel_angle.w_ang1 = wheel_angle.w_ang1 + (mu_1*wheel_velocities.w1_vel);
        // wheel_angle.w_ang2 = wheel_angle.w_ang2 + (mu_1*wheel_velocities.w2_vel);

       //Get the current_configuration of the robot using forward kinematics function
        current_config = update_config.forward_kinematics(wheel_angle);
        
        

        x = current_config.x_config;
        y = current_config.y_config;
        theta = current_config.theta_config;

        // distance_1 = sqrt(pow(current_config.x_config - x_m.at(0), 2) + pow(current_config.y_config - y_m.at(0), 2));
        // distance_2 = sqrt(pow(current_config.x_config - x_m.at(1), 2) + pow(current_config.y_config - y_m.at(1), 2));
        // distance_3 = sqrt(pow(current_config.x_config - x_m.at(2), 2) + pow(current_config.y_config - y_m.at(2), 2));

        // // ROS_WARN("distance_1 %f", distance_1);
        // // distance_1 = 0.1;
        // if(distance_1 <= 0.148){
        //     ROS_INFO_STREAM("state reached");
        //     x = old_x;
        //     y = old_y;
        //     theta = old_theta;
        // }
        // else{
        //     current_config = update_config.forward_kinematics(wheel_angle);
        //     x = current_config.x_config;
        //     y = current_config.y_config;
        //     theta = current_config.theta_config;

        // }
        
        // if(distance_2 <= 0.148){
        //     ROS_INFO_STREAM("state reached");
        //     x = old_x;
        //     y = old_y;
        //     theta = old_theta;
        // }
        // else{
        //     current_config = update_config.forward_kinematics(wheel_angle);
        //     x = current_config.x_config;
        //     y = current_config.y_config;
        //     theta = current_config.theta_config;

        // }

        // if(distance_3 <= 0.148){
        //     ROS_INFO_STREAM("state reached");
        //     x = old_x;
        //     y = old_y;
        //     theta = old_theta;
        // }
        // else{
        //     current_config = update_config.forward_kinematics(wheel_angle);
        //     x = current_config.x_config;
        //     y = current_config.y_config;
        //     theta = current_config.theta_config;

        // }        

       
        //broadcast the transform between nusim and red-base_footprint
        geometry_msgs::TransformStamped transformStamped;
        transformStamped.header.stamp = ros::Time::now();
        transformStamped.header.frame_id = "world";
        transformStamped.child_frame_id = "red-base_footprint";
        transformStamped.transform.translation.x = x;
        transformStamped.transform.translation.y = y;
        transformStamped.transform.translation.z = 0;
        tf2::Quaternion q;
        q.setRPY(0, 0, theta);
        transformStamped.transform.rotation.x = q.x();
        transformStamped.transform.rotation.y = q.y();
        transformStamped.transform.rotation.z = q.z();
        transformStamped.transform.rotation.w = q.w();
        broadcaster.sendTransform(transformStamped);


        //specify path traced by red robot
        geometry_msgs::PoseStamped pose1;
        pose1.header.stamp = ros::Time::now();
        pose1.header.frame_id = "red-base_footprint";
        pose1.pose.position.x = x;
        pose1.pose.position.y = y;
        pose1.pose.position.z = 0.0;

        path1.header.stamp = ros::Time::now();
        path1.header.frame_id = "world";
        path1.poses.push_back(pose1);
       


        //specify path traced by blue robot
        geometry_msgs::PoseStamped pose2;
        pose2.header.stamp = ros::Time::now();
        pose2.header.frame_id = "blue-base_footprint";
        pose2.pose.position.x = x;
        pose2.pose.position.y = y;
        pose2.pose.position.z = 0.0;

        path2.header.stamp = ros::Time::now();
        path2.header.frame_id = "world";
        path2.poses.push_back(pose2);


        //specify path traced by blue robot
        geometry_msgs::PoseStamped pose3;
        pose3.header.stamp = ros::Time::now();
        pose3.header.frame_id = "green-base_footprint";
        pose3.pose.position.x = x;
        pose3.pose.position.y = y;
        pose3.pose.position.z = 0.0;

        path3.header.stamp = ros::Time::now();
        path3.header.frame_id = "world";
        path3.poses.push_back(pose3);




        
        
        //Generate Laser Scan
        ros::Time scan_time = ros::Time::now();
   

        //populate the LaserScan message
        sensor_msgs::LaserScan scan;
        scan.header.stamp = scan_time;
        scan.header.frame_id = "red-base_scan";
        scan.angle_min = 0.0;
        scan.angle_max = 6.28319;
        scan.angle_increment = 6.28319 / num_readings;
        // scan.time_increment = 0.001;
        scan.range_min = 0.120;
        scan.range_max = 3.5;
    
        scan.ranges.resize(num_readings);

        // std::vector <double> a(num_readings);


        for (int i = 0; i < num_readings; i++){
            double min_distance{0.0};
       
            // for(int j = 0; j < num_markers; j++){
            //     // std::vector <double> a(num_readings);

                
            //     //compute points in robot frame
            //     x_r = 0.0;
            //     y_r = 0.0;
            //     x_max = (max_range*cos(theta_range)); // x = rcos(theta)
            //     y_max = (max_range*sin(theta_range)); // y = rsin(theta)
                
            //     // ROS_WARN("points in robot frame");
                
            //     //transformation of robot wrt world
            //     turtlelib::Transform2D Twr{turtlelib::Vector2D{x, y}, theta};
            //     turtlelib::Transform2D Trw = Twr.inv();
            
            //     //Transform of obstacles wrt world
            //     turtlelib::Transform2D Two{turtlelib::Vector2D{x_m.at(j), y_m.at(j)}, 0.0};

            //     // Transform from obstacles wrt robot
            //     turtlelib::Transform2D Tro = Trw*Two;
            //     turtlelib::Transform2D Tor = Tro.inv();
             
            
            //     turtlelib::Vector2D V_op;
            //     V_op = Tor(turtlelib::Vector2D{x_r, y_r});

            //     turtlelib::Vector2D V_opmax;
            //     V_opmax = Tor(turtlelib::Vector2D{x_max, y_max});
                
            //     //get points in obstacle frame
            //     x_p = V_op.x;
            //     y_p = V_op.y;

            //     //get points in obstacle frame
            //     x_pmax = V_opmax.x;
            //     y_pmax = V_opmax.y;

            //     //calculate distance from point on robot to the max point
            //     d_x = x_pmax - x_p;
            //     d_y = y_pmax - y_p;

            //     d_r = distance_fn(d_x, d_y);
            //     // ROS_WARN("x_p: %f", x_p);
            //     // ROS_WARN("y_p: %f", y_p);
            //     // ROS_WARN("x_pmax: %f", x_pmax);
            //     // ROS_WARN("y_pmax: %f", y_pmax);

            //     // ROS_WARN("d_r: %f", d_r);

            //     delta = ((x_p*y_pmax) - (x_pmax*y_p));

            //     //determine points of intersection
                
            //     sgn_dy = sgn_fn(d_y);
            //     mod_dy = mod_fn(d_y);


                
                

            //     discriminant = (pow(radius, 2)*pow(d_r, 2) - pow(delta, 2));
            //     // ROS_WARN("got till discriminant");
            //     // ROS_WARN("discrimant value: %f", discriminant);
                
            //     //store intersection points in array
            //     if(discriminant > 0.0){
            //         x_int_neg = ((delta*d_y - (sgn_dy*d_x*sqrt(pow(radius, 2)*pow(d_r, 2) - pow(delta, 2))))/pow(d_r, 2));

            //         x_int_pos = ((delta*d_y + (sgn_dy*d_x*sqrt(pow(radius, 2)*pow(d_r, 2) - pow(delta, 2))))/pow(d_r, 2));

            //         y_int_neg = ((-(delta*d_x) - (mod_dy*sqrt(pow(radius, 2)*pow(d_r, 2)) - pow(delta, 2)))/pow(d_r, 2));
            
            //         y_int_pos = ((-(delta*d_x) + (mod_dy*sqrt(pow(radius, 2)*pow(d_r, 2)) - pow(delta, 2)))/pow(d_r, 2));

                    
            //         //pos
            //         //calculate transform wrt obstacles for intersection points - pos
            //         turtlelib::Transform2D T_o_intpos{turtlelib::Vector2D{x_int_pos, y_int_pos}, 0.0};



            //         //transform points back to robot frame
            //         turtlelib::Transform2D T_r_intpos = Tro*T_o_intpos;

            //         //get intersection points in robot frame
            //         V_rint_pos = T_r_intpos.translation();

            //         //calculate distance from robot to the intersection point in robot frame
            //         d_xint_pos = V_rint_pos.x - x_r;
            //         d_yint_pos = V_rint_pos.y - y_r;
                    
                    
            //         //neg
            //         //calculate transform wrt obstacles for intersection points - pos
            //         turtlelib::Transform2D T_o_intneg{turtlelib::Vector2D{x_int_neg, y_int_neg}, 0.0};



            //         //transform points back to robot frame
            //         turtlelib::Transform2D T_r_intneg = Tro*T_o_intneg;

            //         //get intersection points in robot frame
            //         V_rint_neg = T_r_intneg.translation();

            //         //calculate distance from robot to the intersection point in robot frame
            //         d_xint_neg = V_rint_neg.x - x_r;
            //         d_yint_neg = V_rint_neg.y - y_r;


            //         intersection_distance_neg = distance_fn(d_xint_neg, d_yint_neg);
            //         intersection_distance_pos = distance_fn(d_xint_pos, d_yint_pos);

            //         ROS_WARN("intersection distance_pos %f", intersection_distance_pos);
            //         ROS_WARN("intersection distance_pos %f", intersection_distance_neg);

            //         min_distance = std::min(intersection_distance_neg, intersection_distance_pos);
                    
            //         ROS_WARN("min_distance: %f", min_distance);
                    
            //     }
            // }

            // walls
            for(int k = 0; k < 4; k++){
                //compute points in robot frame
                x_r = 0.0; // x1
                y_r = 0.0; // y1
                x_max = (max_range*cos(theta_range + theta)); // x2
                y_max = (max_range*sin(theta_range + theta)); // y2

                //compute transformation of robot to world
                // turtlelib::Transform2D Twr{turtlelib::Vector2D{x, y}, theta};
                // turtlelib::Transform2D Trw = Twr.inv();
                
                // //x_max and y_max in world frame
                // turtlelib::Vector2D V_world;
                // V_world = Twr(turtlelib::Vector2D{x_max, y_max});

                //position of robot in world frame
                x_1 = x;
                y_1 = y;
                
                //position of max point in world frame
                x_2 = x + x_max;
                y_2 = y + y_max;
                
                // ROS_WARN("x_2: %f", x_2);
                // ROS_WARN("y_2: %f", y_2);

                


                // Transform to walls wrt robot
                // turtlelib::Transform2D Trwall = Trw*Twwall;
                // turtlelib::Transform2D Twallr = Trwall.inv();

                //wall points in world frame
                if(k == 0){
                    x_3 = 3.0;
                    y_3 = 6.1/2.0;
                    x_4 = 3.0;
                    y_4 = -6.1/2.0;
                }

                if(k == 1){
                    x_3 = -3.0;
                    y_3 = 6.1/2.0;
                    x_4 = -3.0;
                    y_4 = -6.1/2.0;
                }

                if(k == 2){
                    x_3 = 3.0;
                    y_3 = 6.1/2.0;
                    x_4 = -3.0;
                    y_4 = 6.1/2.0;
                }

                if(k == 3){
                    x_3 = 3.0;
                    y_3 = -6.1/2.0;
                    x_4 = -3.0;
                    y_4 = -6.1/2.0;
                }

                // ROS_WARN("x_3: %f", x_3);
                // ROS_WARN("y_3: %f", y_3);

                // ROS_WARN("x_4: %f", x_4);
                // ROS_WARN("y_4: %f", y_4);


              
                //calculate intersection in world frame

                d_new = ((x_1 - x_2)*(y_3 - y_4) - (y_1 - y_2)*(x_3 - x_4));

                p_x = (((x_1*y_2 - y_1*x_2)*(x_3 - y_4) - (x_1 - x_2)*(x_3*y_4 - y_3*x_4))/d_new);

                p_y = (((x_1*y_2 - y_1*x_2)*(y_3 - y_4) - (y_1 - y_2)*(x_3*y_4 - y_3*x_4))/d_new);

                
                // ROS_WARN("p_x: %f", p_x);
                // ROS_WARN("p_y: %f", p_y);




                //calculate intersection distance
                d_x_new = p_x - x_1;
                d_y_new = p_y - y_1;

                intersection_distance_wall = distance_fn(d_x_new, d_y_new);



        
            }

            // if(min_distance < intersection_distance_wall){
            //     scan.ranges[i] = min_distance;

            // }
            // else{
            //     scan.ranges[i] = intersection_distance_wall;
            // }
            scan.ranges[i] = intersection_distance_wall;

            // ROS_WARN("scan.ranges[i]: %f", scan.ranges[i]);
            theta_range += scan.angle_increment; 

        
        
        
        }

    


            



         // marker arrays noise
        obstacle_noise = m_n(get_random());
        
        visualization_msgs::MarkerArray marker_array_noise;


        //publish obstacles with noise
        for (int i = 0; i < num_markers; i++){
            //Transformation of robot wrt world
            turtlelib::Transform2D Twr{turtlelib::Vector2D{x, y}, theta};
            turtlelib::Transform2D Trw = Twr.inv();
            
            //Transform of markers wrt world
            turtlelib::Transform2D Two{turtlelib::Vector2D{x_m.at(i), y_m.at(i)}, 0.0};

            //Transform from markers wrt robot
            turtlelib::Transform2D Tro = Trw*Two;

            V_rel = Tro.translation(); 


            visualization_msgs::Marker marker_noise;
            marker_noise.header.frame_id = "red-base_footprint";
            marker_noise.header.stamp = ros::Time::now();
            marker_noise.ns = "obstacles_noise";
            marker_noise.id = i;
            marker_noise.type = visualization_msgs::Marker:: CYLINDER;
            marker_noise.action = visualization_msgs::Marker::ADD;
            marker_noise.pose.position.x = V_rel.x + obstacle_noise;
            marker_noise.pose.position.y = V_rel.y + obstacle_noise;
            marker_noise.pose.position.z = 0;
            marker_noise.pose.orientation.x = 0.0;
            marker_noise.pose.orientation.y = 0.0;
            marker_noise.pose.orientation.z = 0.0;
            marker_noise.pose.orientation.w = 1.0;
            marker_noise.scale.x = radius*2;
            marker_noise.scale.y = radius*2;
            marker_noise.scale.z = 0.25;
            marker_noise.color.a = 1.0;
            marker_noise.color.r = 0.0;
            marker_noise.color.g = 1.0;
            marker_noise.color.b = 0.0;
            marker_noise.lifetime = ros::Duration();
            
            marker_array_noise.markers.push_back(marker_noise);

    }
    
        fake_pub.publish(marker_array_noise);
        laser_pub.publish(scan);

            
        //publish sensor_data on red/sensor_data topic
        sensor_pub.publish(sensor_data);
            

        //publish path
        path_pub1.publish(path1);
        path_pub2.publish(path2);
        path_pub3.publish(path3);


        // old_x = current_config.x_config;
        // old_y = current_config.y_config;
        // old_theta = current_config.theta_config;
        


        ros::spinOnce();


        r.sleep();
    }
    return 0;
}