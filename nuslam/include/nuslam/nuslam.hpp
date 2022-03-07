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
        int m; //number of obstacles
        int n; //size of matrix
        arma::mat prev_state_vector;
        arma::mat covariance;
        arma::mat state_vector;
        arma::mat q_mat;
        arma::mat r_mat;
        int r;
        int q;
        double init_x_pos;
        double init_y_pos; 
        double init_theta_pos;
        arma::mat m_vec;
        // arma::mat h_2;
        
        
    

    public:
    
        Estimate2d(int m, int n, int r_noise, int q_noise, double init_theta_pos, double init_x_pos, double init_y_pos);

        //calculate state vector (zeta)
        arma::mat updated_state_vector(turtlelib::Twist2D u, int n);

        //calculate A matrix
        arma::mat calculate_A_matrix(turtlelib::Twist2D u, int n);

        //function to compute H
        arma::mat calculate_h(int i);

        // //calculate r matrix
        // arma::mat calculate_r_mat();

        // //calculate q matrix
        // arma::mat calculate_q_mat();

        //calculate z
        arma::mat calculate_z(double x, double y);

        //calculate z_hat
        arma::mat calculate_z_hat(int i);

        //get covariance
        arma::mat get_covariance();

        //get state_vector zeta
        arma::mat get_state_vector();

        //get previous state
        arma::mat get_prev_state_vector();

        //get q_matrix
        arma::mat get_q_matrix();

        //get r_matrix
        arma::mat get_r_matrix();

        void init_fn(std::vector <double> x, std::vector <double> y);
    
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