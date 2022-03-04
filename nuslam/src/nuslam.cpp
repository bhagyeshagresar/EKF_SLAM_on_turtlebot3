#include "nuslam/nuslam.hpp"
#include "turtlelib/rigid2d.hpp"
#include "turtlelib/diff_drive.hpp"
#include <armadillo>
#include "visualization_msgs/MarkerArray.h"



namespace slamlib{
    
    //default constructor
    Estimate2d::Estimate2d(int a, int b)
        :m{a}, 
         n{b},
         prev_state_vector(1, n), 
         covariance(n, n), 
         state_vector(n, 1), 
         q_mat(n, n), 
         r_mat(2, 2), 
         r{0}, 
         q{0} 
         {
             
         }


   

    //calculate state vector (zeta)
    arma::mat Estimate2d::updated_state_vector(turtlelib::Twist2D u){
        
        arma::mat state_vector(1, n);

        if (u.theta_dot == 0){
            //fill the state vector 
            state_vector(0, 0) = prev_state_vector(0, 0) + 0.0;
            state_vector(1, 0) = prev_state_vector(1, 0) + (u.x_dot*cos(prev_state_vector(0, 0)));
            state_vector(2, 0) = prev_state_vector(2, 0) + (u.x_dot*sin(prev_state_vector(0, 0)));

        
            return state_vector;
        }
        else{
            state_vector(0, 0) = prev_state_vector(0, 0) + u.theta_dot;
            state_vector(1, 0) = prev_state_vector(1, 0) + (-(u.x_dot/u.theta_dot)*sin(prev_state_vector(0, 0))) + ((u.x_dot/u.theta_dot)*sin(prev_state_vector(0, 0) + u.theta_dot));
            state_vector(2, 0) = prev_state_vector(2, 0) + ((u.x_dot/u.theta_dot)*cos(prev_state_vector(0, 0))) - ((u.x_dot/u.theta_dot)*cos(prev_state_vector(0, 0) + u.theta_dot));

            return state_vector;


        }
    }


    //calculate A matrix
    arma::mat Estimate2d::calculate_A_matrix(turtlelib::Twist2D u){
        if (u.theta_dot == 0.0)
        {   
            //unit matrix
            arma::mat a_1;
            a_1.eye(n, n);

            //second term
            arma::mat a_2(n, n);
            
            a_2(1, 0) = -u.x_dot*sin(prev_state_vector(0, 2));
            a_2(2, 0) = u.x_dot*cos(prev_state_vector(0, 2));
            arma::mat a_3 = a_1 + a_2;


            return a_3;

        }

        else{
            //second term
            arma::mat a_1;
            a_1.eye(n,n);
            arma::mat a_2(n, n);
          
            a_2(1, 0) = -((u.x_dot/u.theta_dot)*cos(prev_state_vector(0, 2))) + (u.x_dot/u.theta_dot)*cos(prev_state_vector(0, 2)) + u.theta_dot;
            a_2(2, 0) = -((u.x_dot/u.theta_dot)*sin(prev_state_vector(0, 2))) + (u.x_dot/u.theta_dot)*sin(prev_state_vector(0, 2)) + u.theta_dot;
            arma::mat a_3 = a_1 + a_2;
            
            return a_3;
        }        

    }
    
    
    
    //function to compute H
    arma::mat Estimate2d::calculate_h(arma::mat m_vec){
        double d_x{0.0};
        double d_y{0.0};
        double d{0.0};
        double d_root{0.0};

        d_x = m_vec(0,0) - state_vector(0, 0);
        d_y = m_vec(0,1) - state_vector(0, 1);
        d = ((pow(d_x, 2) + pow(d_y, 2)));
        d_root = sqrt(d);

        arma::mat h_2(2, n);
        h_2(1,0) = -1;
        h_2(0,1) = -(d_x/d_root);
        h_2(0, 2) = -(d_y/d_root);
        h_2(1, 1) = (d_y/d_root);
        h_2(1, 2) = -(d_x/d_root);
        h_2(0, 6) = (d_x/d_root);
        h_2(0, 7) = (d_y/d_root);
        h_2(1, 6) = -(d_y/d_root);
        h_2(1, 7) = (d_x/d_root);

        return h_2;

    
    }


    //calculate r matrix
    arma::mat Estimate2d::calculate_r_mat(int r){
        arma::mat r_mat(2, 2);
        r_mat(0, 0) = r;
        r_mat(1, 1) = r;
        return r_mat;
    }

    
    //calculate q matrix
    arma::mat Estimate2d::calculate_q_mat(int q){
        arma::mat q_mat(n, n);
        q_mat(0, 0) = q;
        q_mat(1, 1) = q;
        q_mat(2, 2) = q;
        return q_mat;
    }

    
    //calculate z
    arma::mat Estimate2d::calculate_z(double x, double y){
        double r_j{0.0};
        double phi{0.0};
        arma::mat h(2, 1);
        // for(int i = 0; i < m; i++){
        r_j = sqrt(pow(x, 2) + pow(y, 2));
        phi = atan2(y, x);
        h(0, 0) = r_j;
        h(0, 1) = phi;
        return h;
    }

    
    //calculate z_hat
    arma::mat Estimate2d::calculate_z_hat(int i){
        double r_j{0.0};
        double phi{0.0};
        arma::mat h(2, 1);

        r_j = sqrt(pow(state_vector(0, 3+(2*i)) - state_vector(0,0), 2) + pow(state_vector(0, 4+(2*i)) - state_vector(0,1), 2));
        phi = atan2(state_vector(0, 4+(2*i)) - state_vector(0,1), state_vector(0, 3+(2*i)) - state_vector(0,0)) - state_vector(0,2);

        h(0, 0) = r_j;
        h(0, 1) = phi;
        
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

    
    //set r value
    void Estimate2d::set_r(int a){
        r = a;
    }

    //set q value
    void Estimate2d::set_q(int a){
        q = a;
    }

    

   

}
    
    






