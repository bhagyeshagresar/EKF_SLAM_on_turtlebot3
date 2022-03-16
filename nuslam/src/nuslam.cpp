#include "nuslam/nuslam.hpp"
#include "turtlelib/rigid2d.hpp"
#include "turtlelib/diff_drive.hpp"
#include <armadillo>
#include "visualization_msgs/MarkerArray.h"
#include <ros/ros.h>
#include <ros/console.h>


namespace slamlib{
    
    //default constructor
    Estimate2d::Estimate2d(int m, int n, double r, double q)
        :m{m}, 
         n{n},
        //  prev_state_vector(9, 1, arma::fill::zeros),
         covariance(9, 9, arma::fill::zeros),
         state_vector(9, 1, arma::fill::zeros), 
         q_mat(9, 9, arma::fill::zeros), 
         r_mat(2, 2, arma::fill::zeros), 
         r{r}, 
         q{q},
         m_vec(2, 1, arma::fill::zeros)
         { 
            // prev_state_vector(0, 0) = 0;
            // prev_state_vector(1, 0) = 0;
            // prev_state_vector(2, 0) = 0;
            covariance(3, 3) = 10000000;
            covariance(4, 4) = 10000000;
            covariance(5, 5) = 10000000;
            covariance(6, 6) = 10000000;
            covariance(7, 7) = 10000000;
            covariance(8, 8) = 10000000;
            std::cout << "constructor test" << std::endl;
            r_mat(0, 0) = r;
            r_mat(1, 1) = r;
            std::cout << "constructor test" << std::endl;
            q_mat(0, 0) = q;
            q_mat(1, 1) = q;
            q_mat(2, 2) = q;
            std::cout << "constructor test" << std::endl;
          
                
         }


   

    //calculate state vector (zeta)
    arma::mat Estimate2d::updated_state_vector(turtlelib::Twist2D u, arma::mat prev_state_vector){

        if (turtlelib::almost_equal(u.theta_dot, 0.0)){
            //fill the state vector 
            state_vector(0, 0) = turtlelib::normalize_angle(prev_state_vector(0, 0));
            state_vector(1, 0) = prev_state_vector(1, 0) + (u.x_dot*cos(prev_state_vector(0, 0)));
            state_vector(2, 0) = prev_state_vector(2, 0) + (u.x_dot*sin(prev_state_vector(0, 0)));
            


        
        }
        else{
            double delta = u.x_dot/u.theta_dot;
            state_vector(0, 0) = turtlelib::normalize_angle(prev_state_vector(0, 0) + u.theta_dot);
            state_vector(1, 0) = prev_state_vector(1, 0) + (-(delta)*sin(prev_state_vector(0, 0))) + ((delta)*sin(prev_state_vector(0, 0) + u.theta_dot));
            state_vector(2, 0) = prev_state_vector(2, 0) + ((delta)*cos(prev_state_vector(0, 0))) - ((delta)*cos(prev_state_vector(0, 0) + u.theta_dot));
           



        }
        return state_vector;

    }


    //calculate A matrix
    arma::mat Estimate2d::calculate_A_matrix(turtlelib::Twist2D u, int n, arma::mat prev_state_vector){
        arma::mat a_1 = arma::eye(9, 9);
        arma::mat a_2(9, 9, arma::fill::zeros);
        arma::mat a_3(9, 9, arma::fill::zeros);
        
        if (turtlelib::almost_equal(u.theta_dot, 0.0))
        {   
            //unit matrix
           

            //second term
            
            a_2(1, 0) = -u.x_dot*sin(prev_state_vector(0, 0));
            a_2(2, 0) = u.x_dot*cos(prev_state_vector(0, 0));
            a_3 = a_1 + a_2;



        }

        else{
            //second term
            double delta_2 = u.x_dot/u.theta_dot;
          
            a_2(1, 0) = -(delta_2)*cos(prev_state_vector(0, 0)) + (delta_2)*cos(prev_state_vector(0, 0) + u.theta_dot);
            a_2(2, 0) = -(delta_2)*sin(prev_state_vector(0, 0)) + (delta_2)*sin(prev_state_vector(0, 0) + u.theta_dot);
            a_3 = a_1 + a_2;
            
        }
        return a_3;        
    }
    
    
    
    //function to compute H
    arma::mat Estimate2d::calculate_h(int i){
        double d_x{0.0};
        double d_y{0.0};
        double d{0.0};
        double d_root{0.0};

        d_x = state_vector(3+(2*i), 0) - state_vector(1, 0);
        d_y = state_vector(4+(2*i), 0) - state_vector(2, 0);
        d = ((pow(d_x, 2) + pow(d_y, 2)));
        // d_root = sqrt(d);

        arma::mat h_2(2, 9, arma::fill::zeros);
        if(!turtlelib::almost_equal(d, 0.0)){
            h_2(1,0) = -1;
            h_2(0,1) = -(d_x/sqrt(d));
            h_2(0, 2) = -(d_y/sqrt(d));
            h_2(1, 1) = (d_y/d);
            h_2(1, 2) = -(d_x/d);
            h_2(0, 3+(2*i)) = (d_x/sqrt(d));
            h_2(0, 4+(2*i)) = (d_y/sqrt(d));
            h_2(1, 3+(2*i)) = -(d_y/d);
            h_2(1, 4+(2*i)) = (d_x/d);
        }
        
        // h_2.print("check h calculations");
        // std::cout << "d_root %f" << d_root << std::endl;


        return h_2;

    
    }


    // //calculate r matrix
    // arma::mat Estimate2d::calculate_r_mat(){
    //     r_mat(0, 0) = r;
    //     r_mat(1, 1) = r;
    //     return r_mat;
    // }

    
    // //calculate q matrix
    // arma::mat Estimate2d::calculate_q_mat(){
        
    //     q_mat(0, 0) = q;
    //     q_mat(1, 1) = q;
    //     q_mat(2, 2) = q;
    //     return q_mat;
    // }

    
    //calculate z
    arma::mat Estimate2d::calculate_z(double g, double y){
        arma::mat h(2, 1);
        h(0, 0) = g;
        h(1, 0) = turtlelib::normalize_angle(y);
        return h;
    }

    
    //calculate z_hat
    arma::mat Estimate2d::calculate_z_hat(int i){
       
        arma::mat z_hat(2, 1, arma::fill::zeros);

        z_hat(0, 0) = sqrt(pow(state_vector(3+(2*i), 0) - state_vector(1,0), 2) + pow(state_vector(4+(2*i), 0) - state_vector(2,0), 2));
        z_hat(1, 0) = turtlelib::normalize_angle(atan2(state_vector(4+(2*i), 0) - state_vector(2,0), state_vector(3+(2*i),0) - state_vector(1,0)) - state_vector(0,0));

       
        
        return z_hat;
    }
    
    
    //get covariance
    arma::mat Estimate2d::get_covariance(){
        return covariance;
    }

    //get state_vector zeta
    arma::mat Estimate2d::get_state_vector(){
        return state_vector;
    }
    
    
    //get previous state
    // arma::mat Estimate2d::get_prev_state_vector(){
    //     return prev_state_vector;
    // }

    //get q_matrix
    arma::mat Estimate2d::get_q_matrix(){
        return q_mat;
    }

    //get r_matrix
    arma::mat Estimate2d::get_r_matrix(){
        return r_mat;
    }

    void Estimate2d::init_fn(arma::mat temp_vec, int m, arma::mat prev_state_vector)
    {
       
      
        // arma::mat m_vec(2, 1);
        
        for(int i = 0; i < 3; i++){
            
            m_vec(0, 0) = (prev_state_vector(1, 0)) + temp_vec(2*i, 0)*cos(turtlelib::normalize_angle(temp_vec((2*i)+1, 0) + prev_state_vector(0, 0)));
            m_vec(1, 0) = (prev_state_vector(2, 0)) + temp_vec(2*i, 0)*sin(turtlelib::normalize_angle(temp_vec((2*i)+1, 0) + prev_state_vector(0, 0)));

            state_vector(3+(2*i), 0) = m_vec(0, 0);
            state_vector(4+(2*i), 0) = m_vec(1, 0);
            // ROS_WARN("r: ")
        }

        // state_vector.print("init state_vector");

        state_vector.print("state vector");
        // covariance.print("covariance matrix");
        // q_mat.print("q_matrix");
        // r_mat.print("r_matrix");
        prev_state_vector.print("prev state vector");
        // h_2.print("H matrix");

    }


    // void Estimate2d::calculate_range_bearing(double x, double y){
       
    //     r_j = sqrt(pow(x, 2) + pow(y, 2));
    //     phi = atan2(y, x);

    // }

    // double Estimate2d::get_rj(){
    //     return r_j;
    // }

    // double Estimate2d::get_phi(){
    //     return phi;
    // }

    // void Estimate2d::set_prev_vector(arma::mat a){
    //     prev_state_vector = a;
    // }

    // //set r value
    // void Estimate2d::set_r(int a){
    //     r = a;
    // }

    // //set q value
    // void Estimate2d::set_q(int a){
    //     q = a;
    // }





   

}