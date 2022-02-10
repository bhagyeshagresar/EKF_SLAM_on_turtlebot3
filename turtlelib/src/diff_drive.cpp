#include "turtlelib/rigid2d.hpp"
#include "turtlelib/diff_drive.hpp"
#include <ros/ros.h>
#include <ros/console.h>




namespace turtlelib
{

    DiffDrive::DiffDrive()
        :w_ang{0.0, 0.0}, w_vel{0.0, 0.0}, q{0.0, 0.0, 0.0}
        {
            
        }
    
    // DiffDrive::DiffDrive(Wheel_angles w_ang, Wheels_vel w_vel, Configuration q)
    //     :w_ang{w_ang.w_ang1, w_ang.w_ang2}, w_vel{w_vel.w1_vel, w_vel.w2_vel}, q{q.x_config, q.y_config, q.theta_config}
    //     {
            
    //     }


    DiffDrive::DiffDrive(Configuration q)
        :q{q.x_config, q.y_config, q.theta_config}
        {
            
        }

   

    Configuration DiffDrive::forward_kinematics(Wheel_angles new_wheel_ang){
        Twist2D V_fwd;
        // Wheels_vel w_vel;
        // Wheel_angles old_wheel_ang;
        // Vector2D new_vector_config;
        
        // Configuration config;
            



        //update wheel velocities
        w_vel.w1_vel = new_wheel_ang.w_ang1 - w_ang.w_ang1;
        w_vel.w2_vel = new_wheel_ang.w_ang2 - w_ang.w_ang2;
        // std::cout << "fwd w_vel1: " << w_vel.w1_vel << std::endl;
        // std::cout << "fwd w_vel2: " << w_vel.w2_vel << std::endl;

        
        
        //update wheel angles
        w_ang.w_ang1 = new_wheel_ang.w_ang1;
        w_ang.w_ang2 = new_wheel_ang.w_ang2;

        // std::cout << "old_wheel_ang.w_ang1 " << old_wheel_ang.w_ang1 << std::endl;
        // std::cout << "old_wheel_ang.w_ang2 " << old_wheel_ang.w_ang2 << std::endl;

        

        //Calculate vx and thetadot
        V_fwd.x_dot = (r*(w_vel.w1_vel + w_vel.w2_vel))/2;
        V_fwd.theta_dot = (r*(w_vel.w2_vel - w_vel.w1_vel))/(2*d);
        std::cout << "r: " << r << std::endl;
        std::cout << "d: " << d << std::endl;
        

        
        std::cout << "V_fwd.x_dot " << V_fwd.x_dot << std::endl;
        std::cout << "V_fwd.theta_dot " << V_fwd.theta_dot << std::endl;


        Transform2D T_world_robot{Vector2D{q.x_config, q.y_config}, q.theta_config};
        Transform2D T_old_new = integrate_twist(V_fwd);

        Transform2D Twnew = T_world_robot*T_old_new;

        std::cout << "T_world_robot " << T_world_robot << std::endl;
        std::cout << "V_fwd.x_dot " << V_fwd.x_dot << std::endl;
        std::cout << "V_fwd.x_dot " << V_fwd.x_dot << std::endl;




        Vector2D new_vector_config = Twnew.translation();
        double new_theta_config = Twnew.rotation();

        std::cout << "new_vector_config.x" << new_vector_config.x << std::endl;
        std::cout << "new_vector_config.y " << new_vector_config.y << std::endl;
        std::cout << "new_vector_config.theta_config " << new_theta_config << std::endl;



        q.x_config = new_vector_config.x;
        q.y_config = new_vector_config.y;
        q.theta_config = new_theta_config;

        std::cout << "config.x_config " << q.x_config << std::endl;
        std::cout << "config.y_config " << q.y_config << std::endl;
        std::cout << "config.theta_config " << q.theta_config << std::endl;




        return q;

    }

    Wheels_vel DiffDrive::inverse_kinematics(Twist2D V){
        Wheels_vel w_vel;

        w_vel.w1_vel = (1/r)*((-d*V.theta_dot) + V.x_dot);
        w_vel.w2_vel = (1/r)*((d*V.theta_dot) + V.x_dot);
        
        std::cout << "w_vel.w1_vel " << w_vel.w1_vel << std::endl;
        std::cout << "w_vel.w2_vel " << w_vel.w2_vel << std::endl;

        
        return w_vel;


    }

    double DiffDrive::get_radius(){
        return r;
    }
    
    double DiffDrive::get_length_d(){
        return d;
    }

}

