#include "nuslam/slam.hpp"
#include "turtlelib/rigid2d.hpp"
#include <armadillo>



namespace slamlib{
    
    //default constructor
    Estimate2d::Estimate2d(int a, int b)
        :m{a}, n{b}, prev_state_vector(1, n), covariance(n, n), state_vector(1, n), q(n, n) 
         {
             
         }


    // //initialise function for mx and my
    // double Estimate2d::initialise_fn(turtlelib::Configuration robot_config){
    //     for(int i = 0; i < m; i++){

    //         state_vector(1, ) = (robot_config.x + (calculate_r*cos(calculate_phi+robot_config.theta)));
    //         m_y = (robot_config.y + (calculate_r*sin(calculate_phi+robot_config.theta)));

    // }



    //calculate r
    // double Estimate2d::calculate_r(double x_bar, double y_bar){
    //     double r{0.0};
    //     //need to loop ?
    //     // what is x_bar and y_bar?
    //     r = sqrt(pow(x_bar, 2) + pow(y_bar, 2));
    

    //     return r;
    
    // }

    // //calculate phi
    // double Estimate2d::calculate_phi(double x_bar, double y_bar){
        
    //     double phi{0.0};

    //     phi = atan2(y_bar, x_bar);

    //     return phi;
    // }



    //calculate state vector
    arma::mat <double> Estimate2d::updated_state_vector(turtlelib::Twist2D u){
        
        arma::mat <double> state_vector(1, n);

        if (u.theta_dot == 0){
            //fill the state vector 
            state_vector(0, 0) = prev_state_vector(0, 0) + 0.0;
            state_vector(0, 1) = prev_state_vector(0, 1) + (u.x_dot*cos(robot_config.theta));
            state_vector(0, 2) = prev_state_vector(0, 2) + (u.x_dot*sin(robot_config.theta));

        
            return state_vector;
        }
        else{
            state_vector(0, 0) = prev_state_vector(0, 0) + u.theta_dot;
            state_vector(0, 1) = prev_state_vector(0, 1) + (-(u.x_dot/u.theta_dot)*sin(prev_state_vector(0, 2)) + ((u.x_dot/u.theta_dot)*sin(prev_state_vector(0, 2) + u.theta_dot));
            state_vector(0, 2) = prev_state_vector(0, 2) + ((u.x_dot/u.theta_dot)*cos(prev_state_vector(0, 2)) - ((u.x_dot/u.theta_dot)*cos(prev_state_vector(0, 2) + u.theta_dot));

            return state_vector;


        }
    }


   

  

    //calculate A matrix
    arma::mat calculate_A_matrix(turtlelib::Twist2D u){
        if (theta == 0.0)
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
          
            a_2(1, 0) = -(u.x_dot/u.theta_dot)*cos(prev_state_vector(0, 2)) + (u.x_dot/u.theta_dot)*cos(prev_state_vector(0, 2) + u.theta_dot;
            a_2(2, 0) = -(u.x_dot/u.theta_dot)*sin(prev_state_vector(0, 2)) + (u.x_dot/u.theta_dot)*sin(prev_state_vector(0, 2) + u.theta_dot;
            arma::mat a_3 = a_1 + a_2;
            
            return a_3;
        }        

    }
    


    //function to compute z for update
    arma::mat Estimate2d::calculate_z(int m){
        for(int i = 0; i < m; i++){
            r_j = sqrt(pow(state_vector(0, i+3) - state_vector(0,0), 2) + pow(state_vector(0, i+4) - state_vector(0,1), 2));
            phi = atan2(state_vector(0, i+4) - state_vector(0,1), state_vector(0, i+3) - state_vector(0,0)) - state_vector(0,2);

            arma::mat h(2, 1);
            h(0, 0) = r_j;
            h(0, 1) = phi;
        }
        return h;
    }

    
    
    //function to compute H
    arma::mat Estimate2d::calculate_h(){
        d_x = m_x - state_vector(0, 0);
        d_y = m_y - state_vector(0, 1);
        d = ((pow(d_x, 2) + pow(d_y, 2));
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
    arma::mat Estimate2d::calculate_r(){
        arma::mat r_mat(2, 2);
        r_mat(0, 0) = r;
        r_mat(1, 1) = r;
        return r_mat;
    }
    
    
    //prediciton function
        
    //step 1 - calculate zeta
    arma::mat zeta = calculate_state_vector(turtlelib::Twist2D u);

    //step 2 - calculate sigma_t_
        
   
       
    // arma::mat covariance = (a*covariance*a_transpose) + q);

    arma::mat Estimate2d::get_covariance(){
        return covariance;
    }

    arma::mat Estimate2d::get_state_vector(){
        return state_vector;
    }
    
    arma::mat Estimate2d::get_prev_state_vector(){
        return prev_state_vector;
    }





}
    
    






