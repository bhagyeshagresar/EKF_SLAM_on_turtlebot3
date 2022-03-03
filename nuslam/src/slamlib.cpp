#include "nuslam/slam.hpp"
#include "turtlelib/rigid2d.hpp"
#include <armadillo>




namespace slamlib{
    
    //default constructor
    Estimate2d::Estimate2d()
        :m{0}, n{0}, prev_state_vector(1, n), covariance(n, n), state_vector(1, n) 
         {
             
         }


    //initialise function for mx and my
    double Estimate2d::initialise_fn(turtlelib::Configuration robot_config){
        for(int i = 0; i < m; i++){
            m_x = (robot_config.x + (calculate_r*cos(calculate_phi+robot_config.theta)));
            m_y = (robot_config.y + (calculate_r*sin(calculate_phi+robot_config.theta)));

    }



    //calculate r
    double Estimate2d::calculate_r(double x_bar, double y_bar){
        double r{0.0};
        //need to loop ?
        // what is x_bar and y_bar?
        r = sqrt(pow(x_bar, 2) + pow(y_bar, 2));
    

        return r;
    
    }

    //calculate phi
    double Estimate2d::calculate_phi(double x_bar, double y_bar){
        
        double phi{0.0};

        phi = atan2(y_bar, x_bar);

        return phi;
    }



    //calculate g(zeta(t-1), u_t)
    arma::mat <double> Estimate2d::calculate_state_vector(turtlelib::Twist2D u){
        
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


   

  

    //calculate A_t
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
            arma::mat a_t = a_1 + a_2;


            return a_t;

        }

        else{
            //second term
            arma::mat a_1;
            a_1.eye(n,n);
            arma::mat a_2(n, n);
          
            a_2(1, 0) = -(u.x_dot/u.theta_dot)*cos(prev_state_vector(0, 2)) + (u.x_dot/u.theta_dot)*cos(prev_state_vector(0, 2) + u.theta_dot);
            a_2(2, 0) = -(u.x_dot/u.theta_dot)*sin(prev_state_vector(0, 2)) + (u.x_dot/u.theta_dot)*sin(prev_state_vector(0, 2) + u.theta_dot);
            arma::mat a_t = a_1 + a_2;
            
            return a_t;
        }        

    }
    
    
    turtlelib::Vector2D Estimate2d::prediction(double prev_theta, turtlelib::Twist2D u){
        
        //step 1 - calculate zeta
        arma::mat zeta = calculate_state_vector(u);

        //step 2 - calculate sigma_t_
        
        //calculate a_t and a_t_transpose
        arma::mat a_t = calculate_A_matrix(u);
        arma::mat a_t_transpose = a_t.t();

        //how to calculate sigma_t-1?
        

        //comput process noise Q
        // arma::mat q(n, n);
        // q.zeros(n, n);
        // q(0, 0) = q_value;

        arma::mat  = A_t*sigma(t-1)*A_t(transpose) + Q
        //what to return???
        return zeta;

    }

    
    update_fn(){
    

    //compute theoretical measurement z_t
    for(int i = 0; i < m; i++){
        for(int j = 0; j < n; i++){
            r_j = sqrt(pow(m_x - robot_config.x, 2) + pow(m_y - robot_config.y, 2));
            phi = atan2(m_y - robot_config.y, m_x - robot_config.x) - robot_config.theta;

            arma::mat h(2, 1);
            h(0, 0) = r_j;
            h(0, 1) = phi;
        }
        arma::mat z = h;
    }

    //compute kalman gain ki

        // a) get sigma using function

        // b) comput Hi using function

        // c)compute transpose of Hi

        // what is R ???

        //calculate ki
    


    //compute posterior state update
        //how to comput z_t_i?
    
    new_zeta = zeta + Ki*(z_t_i - z_hat_i);



    //compute posterior covariance


    }


