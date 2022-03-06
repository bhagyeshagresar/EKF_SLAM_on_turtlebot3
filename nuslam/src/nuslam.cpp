#include "nuslam/nuslam.hpp"
#include "turtlelib/rigid2d.hpp"
#include "turtlelib/diff_drive.hpp"
#include <armadillo>
#include "visualization_msgs/MarkerArray.h"
#include <ros/ros.h>
#include <ros/console.h>


namespace slamlib{
    
    //default constructor
    Estimate2d::Estimate2d(int m, int n, int r_noise, int q_noise, double init_theta_pos, double init_x_pos, double init_y_pos)
        :m{m}, 
         n{n},
         prev_state_vector(n, 1, arma::fill::zeros),
         covariance(n, n, arma::fill::zeros),
         state_vector(n, 1, arma::fill::zeros), 
         q_mat(n, n, arma::fill::zeros), 
         r_mat(2, 2, arma::fill::zeros), 
         r{r_noise}, 
         q{q_noise},
         init_theta_pos{init_theta_pos},
         init_x_pos{init_x_pos},
         init_y_pos{init_y_pos},
         m_vec(2, 1, arma::fill::zeros),
         h_2(2, n, arma::fill::zeros)
         { 
            prev_state_vector(0, 0) = init_theta_pos;
            prev_state_vector(1, 0) = init_x_pos;
            prev_state_vector(2, 0) = init_y_pos;
            covariance(0, 0) = init_theta_pos;
            covariance(1, 1) = init_x_pos;
            covariance(2, 2) = init_y_pos;
            covariance(3, 3) = 100000;
            covariance(4, 4) = 100000;
            covariance(5, 5) = 100000;
            covariance(6, 6) = 100000;
            covariance(7, 7) = 100000;
            covariance(8, 8) = 100000;
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
    arma::mat Estimate2d::updated_state_vector(turtlelib::Twist2D u, int n){
        

        if (u.theta_dot == 0.0){
            //fill the state vector 
            state_vector(0, 0) = prev_state_vector(0, 0) + 0.0;
            state_vector(1, 0) = prev_state_vector(1, 0) + (u.x_dot*cos(prev_state_vector(0, 0)));
            state_vector(2, 0) = prev_state_vector(2, 0) + (u.x_dot*sin(prev_state_vector(0, 0)));

        
        }
        else{
            state_vector(0, 0) = prev_state_vector(0, 0) + u.theta_dot;
            state_vector(1, 0) = prev_state_vector(1, 0) + (-(u.x_dot/u.theta_dot)*sin(prev_state_vector(0, 0))) + ((u.x_dot/u.theta_dot)*sin(prev_state_vector(0, 0) + u.theta_dot));
            state_vector(2, 0) = prev_state_vector(2, 0) + ((u.x_dot/u.theta_dot)*cos(prev_state_vector(0, 0))) - ((u.x_dot/u.theta_dot)*cos(prev_state_vector(0, 0) + u.theta_dot));



        }
        return state_vector;

    }


    //calculate A matrix
    arma::mat Estimate2d::calculate_A_matrix(turtlelib::Twist2D u, int n){
        arma::mat a_1 = arma::eye(n, n);
        arma::mat a_2(n, n, arma::fill::zeros);
        arma::mat a_3(n, n);
        
        if (u.theta_dot == 0.0)
        {   
            //unit matrix
           

            //second term
            
            a_2(1, 0) = -u.x_dot*sin(prev_state_vector(0, 0));
            a_2(2, 0) = u.x_dot*cos(prev_state_vector(0, 0));
            a_3 = a_1 + a_2;



        }

        else{
            //second term
            
          
            a_2(1, 0) = -((u.x_dot/u.theta_dot)*cos(prev_state_vector(0, 0))) + (u.x_dot/u.theta_dot)*cos(prev_state_vector(0, 0)) + u.theta_dot;
            a_2(2, 0) = -((u.x_dot/u.theta_dot)*sin(prev_state_vector(0, 0))) + (u.x_dot/u.theta_dot)*sin(prev_state_vector(0, 0)) + u.theta_dot;
            a_3 = a_1 + a_2;
            
        }
        return a_3;        
    }
    
    
    
    //function to compute H
    arma::mat Estimate2d::calculate_h(){
        double d_x{0.0};
        double d_y{0.0};
        double d{0.0};
        double d_root{0.0};

        d_x = m_vec(0,0) - state_vector(1, 0);
        d_y = m_vec(1,0) - state_vector(2, 0);
        d = ((pow(d_x, 2) + pow(d_y, 2)));
        d_root = sqrt(d);

        // arma::mat h_2(2, n);
        h_2(1,0) = -1;
        h_2(0,1) = -(d_x/d_root);
        h_2(0, 2) = -(d_y/d_root);
        h_2(1, 1) = (d_y/d_root);
        h_2(1, 2) = -(d_x/d_root);
        h_2(0, 6) = (d_x/d_root);
        h_2(0, 7) = (d_y/d_root);
        h_2(1, 6) = -(d_y/d_root);
        h_2(1, 7) = (d_x/d_root);

        std::cout << "d_root %f" << d_root << std::endl;


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
    arma::mat Estimate2d::calculate_z(double x, double y){
        double r_j{0.0};
        double phi{0.0};
        arma::mat h(2, 1, arma::fill::zeros);
        // for(int i = 0; i < m; i++){
        r_j = sqrt(pow(x, 2) + pow(y, 2));
        phi = atan2(y, x);
        h(0, 0) = r_j;
        h(1, 0) = phi;
        return h;
    }

    
    //calculate z_hat
    arma::mat Estimate2d::calculate_z_hat(int i){
        double r_j{0.0};
        double phi{0.0};
        arma::mat h(2, 1);

        r_j = sqrt(pow(state_vector(3+(2*i), 0) - state_vector(1,0), 2) + pow(state_vector(4+(2*i), 0) - state_vector(2,0), 2));
        phi = atan2(state_vector(4+(2*i), 0) - state_vector(2,0), state_vector(3+(2*i),0) - state_vector(1,0)) - state_vector(0,0);

        h(0, 0) = r_j;
        h(1, 0) = phi;
        
        return h;
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
    arma::mat Estimate2d::get_prev_state_vector(){
        return prev_state_vector;
    }

    //get q_matrix
    arma::mat Estimate2d::get_q_matrix(){
        return q_mat;
    }

    //get r_matrix
    arma::mat Estimate2d::get_r_matrix(){
        return r_mat;
    }

    void Estimate2d::init_fn(std::vector <double> x, std::vector <double> y){
        double range{0.0};
        double phi{0.0};
        // arma::mat m_vec(2, 1);
        
        for(int i = 0; i < m; i++){
            range = sqrt(pow(x.at(i), 2) + pow(y.at(i), 2));
            phi = atan2(y.at(i), x.at(i));
            m_vec(0, 0) = (prev_state_vector(1, 0)) + r*cos(phi + prev_state_vector(0, 0));
            m_vec(1, 0) = (prev_state_vector(2, 0)) + r*sin(phi + prev_state_vector(0, 0));

            state_vector(3+(2*i), 0) = m_vec(0, 0);
            state_vector(4+(2*i), 0) = m_vec(1, 0);
        }

        state_vector.print("init state_vector");

        // state_vector.print("state vector");
        // covariance.print("covariance matrix");
        // q_mat.print("q_matrix");
        // r_mat.print("r_matrix");
        // prev_state_vector.print("prev state vector");
        // h_2.print("H matrix");

    }

    
    // //set r value
    // void Estimate2d::set_r(int a){
    //     r = a;
    // }

    // //set q value
    // void Estimate2d::set_q(int a){
    //     q = a;
    // }





   

}
    
    






