#include "turtlelib/rigid2d.hpp"
#include "turtlelib/diff_drive.hpp"
#include <ros/ros.h>
// #include <ros/console.



namespace turtlelib
{

    DiffDrive::DiffDrive()
        :w_ang{0.0, 0.0}, w_vel{0.0, 0.0}, q{0.0, 0.0, 0.0}, d{2.0}, r{1.0}{

        }

    Configuration DiffDrive::forward_kinematics(Wheel_angles new_wheel_ang){
        Twist2D V_fwd;
        Wheels_vel w_vel;
        Wheel_angles old_wheel_ang {0.0, 0.0};
        Vector2D new_vector_config;
        double new_theta_config {0.0};
        Configuration config;

        //update wheel velocities
        w_vel.w1_vel = new_wheel_ang.w_ang1 - old_wheel_ang.w_ang1;
        w_vel.w2_vel = new_wheel_ang.w_ang2 - old_wheel_ang.w_ang2;   

        //update wheel angles
        old_wheel_ang.w_ang1 = new_wheel_ang.w_ang1;
        old_wheel_ang.w_ang2 = new_wheel_ang.w_ang2;
        

        //Calculate vx and thetadot
        V_fwd.x_dot = (r*(w_vel.w1_vel + w_vel.w2_vel))/2;
        V_fwd.theta_dot = (r*(w_vel.w2_vel - w_vel.w1_vel))/(2*d);

        Transform2D T_world_robot{Vector2D{config.x_config, config.y_config}, config.theta_config};
        Transform2D T_old_new = integrate_twist(V_fwd);

        new_vector_config = T_old_new.translation();
        new_theta_config = T_old_new.rotation();

        config.x_config = new_vector_config.x;
        config.y_config = new_vector_config.y;
        config.theta_config = new_theta_config;



        return config;

    }

    Wheels_vel DiffDrive::inverse_kinematics(Twist2D V){
        Wheels_vel w_vel;

        w_vel.w1_vel = (1/r)*((-d*V.theta_dot) + V.x_dot);
        w_vel.w2_vel = (1/r)*((d*V.theta_dot)+ V.x_dot);

        return w_vel;


    }

    double DiffDrive::get_radius(){
        return r;
    }
    
    double DiffDrive::get_length_d(){
        return d;
    }

}

