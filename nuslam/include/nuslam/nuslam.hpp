#ifndef NUSLAM_INCLUDE_GUARD_HPP
#define NUSLAM_INCLUDE_GUARD_HPP


#include "turtlelib/rigid2d.hpp"
#include "turtlelib/diff_drive.hpp"
#include <armadillo>

namespace slamlib
{
    



    class Estimate2d
    {
    private:
        // int m; //number of obstacles
        // int n; //size of matrix
        // arma::mat prev_state_vector;
        arma::mat covariance;
        // arma::mat state_vector;
        arma::mat q_mat;
        arma::mat r_mat;
        double r;
        double q;
        // double init_x_pos;
        // double init_y_pos; 
        // double init_theta_pos;
        arma::mat m_vec;
        arma::mat predict_vector;
        // double r_j;
        // double phi;
        // arma::mat h_2;
        
        
    

    public:
    
        Estimate2d(double r, double q);

        //calculate state vector (zeta)
        arma::mat updated_state_vector(turtlelib::Twist2D u, arma::mat prev_state_vector);

        //calculate A matrix
        arma::mat calculate_A_matrix(turtlelib::Twist2D u);

        //function to compute H
        arma::mat calculate_h(int i);

        // //calculate r matrix
        // arma::mat calculate_r_mat();

        // //calculate q matrix
        // arma::mat calculate_q_mat();

        //calculate z
        arma::mat calculate_z(double g, double y);

        //calculate z_hat
        arma::mat calculate_z_hat(int i);

        //get covariance
        arma::mat get_covariance();

        //get state_vector zeta
        // arma::mat get_state_vector();

        //get previous state
        // arma::mat get_prev_state_vector();

        //get q_matrix
        arma::mat get_q_matrix();

        //get r_matrix
        arma::mat get_r_matrix();

        void init_fn(arma::mat temp_vec);

        // void calculate_range_bearing(double x, double y);

        // double get_rj();

        // double get_phi();

        void set_prev_vector(arma::mat a);

        void set_predict_vector(arma::mat final_vector);

        arma::mat get_predict_vector();
    
        // //set r value
        // void set_r(int a);

        // //set q value
        // void set_q(int a);

        //initalise covariance matrix
        // arma::mat init_covariance(double init_x_pos, double init_y_pos,  double init_theta_pos);

        // //initialise prev_state_vector
        // arma::mat init_prev_state_vector(double init_x_pos, double init_y_pos,  double init_theta_pos);

};

}



#endif
