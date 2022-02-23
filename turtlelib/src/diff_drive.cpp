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
    
    

    DiffDrive::DiffDrive(Configuration q)
        :q{q.x_config, q.y_config, q.theta_config}
        {
            
        }

   

    Configuration DiffDrive::forward_kinematics(Wheel_angles new_wheel_ang){
        Twist2D V_fwd;
       



        //update wheel velocities
        w_vel.w1_vel = new_wheel_ang.w_ang1 - w_ang.w_ang1;
        w_vel.w2_vel = new_wheel_ang.w_ang2 - w_ang.w_ang2;
      
        
        
        //update wheel angles
        w_ang.w_ang1 = new_wheel_ang.w_ang1;
        w_ang.w_ang2 = new_wheel_ang.w_ang2;

       
        //Calculate vx and thetadot
        V_fwd.x_dot = (r*(w_vel.w1_vel + w_vel.w2_vel))/2;
        V_fwd.theta_dot = (r*(w_vel.w2_vel - w_vel.w1_vel))/(2*d);
        
        // Transformation matrix for robot wrt world
        Transform2D T_world_robot{Vector2D{q.x_config, q.y_config}, q.theta_config};
        
        // Transformation matrix for new frame of the robot wrt old frame 
        Transform2D T_old_new = integrate_twist(V_fwd);

        // Transformation matrix for new frame wrt old frame
        Transform2D Twnew = T_world_robot*T_old_new;

        // Compute the current configuration of the robot
        Vector2D new_vector_config = Twnew.translation();
        double new_theta_config = Twnew.rotation();


        q.x_config = new_vector_config.x;
        q.y_config = new_vector_config.y;
        q.theta_config = new_theta_config;

        


        return q;

    }

    Wheels_vel DiffDrive::inverse_kinematics(Twist2D V){
        Wheels_vel w_vel;

        w_vel.w1_vel = (1/r)*((-d*V.theta_dot) + V.x_dot);
        w_vel.w2_vel = (1/r)*((d*V.theta_dot) + V.x_dot);
        
        // std::cout << "w_vel.w1_vel " << w_vel.w1_vel << std::endl;
        // std::cout << "w_vel.w2_vel " << w_vel.w2_vel << std::endl;

        
        return w_vel;


    }

    double DiffDrive::get_radius(){
        return r;
    }
    
    double DiffDrive::get_length_d(){
        return d;
    }

    Configuration DiffDrive::get_config(){
        return q;

    }

    void DiffDrive::set_config(Configuration new_q){
        q.x_config = new_q.x_config;
        q.y_config = new_q.y_config;
        q.theta_config = new_q.theta_config;

    }

}

