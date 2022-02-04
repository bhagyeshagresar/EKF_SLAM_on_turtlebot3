#include "turtlelib/rigid2d.hpp"
#include "diff_drive.hpp"
#include <ros/ros.h>
// #include <ros/console.



namespace turtlelib
{

    DiffDrive::DiffDrive()
        :w{0.0, 0.0}, w_vel{0.0, 0.0}, q{0.0, 0.0, 0.0}, d{2.0}, r{1.0}{

        }

    Twist2D DiffDrive::forward_kinematics(Wheels_vel wheel){
        Twist2D V_fwd;
        

        
        V_fwd.x_dot = (r*(wheel.w1_vel + wheel.w2_vel))/2;

        V_fwd.theta_dot = (r*(wheel.w2_vel - wheel.w1_vel))/(2*d);
        // ROS::ROS_INFO_STREAM("forward twist x_dot", V_fwd.x_dot);
        // ROS::ROS_INFO_STREAM("forward twist theta_dot", V_fwd.theta_dot);

        return V_fwd;


    }

    Wheels_vel DiffDrive::inverse_kinematics(Twist2D V){
        Wheels_vel w_vel;

        w_vel.w1_vel = (1/r)*((-d*V.theta_dot) + V.x_dot);
        w_vel.w2_vel = (1/r)*((d*V.theta_dot)+ V.x_dot);

        return w_vel;


    }
    

}

