#ifndef SLAMLIB_INCLUDE_GUARD_HPP
#define SLAMLIB_INCLUDE_GUARD_HPP


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
        arma::mat <double> prev_state_vector;
        arma::mat <double> covariance;
        arma::mat <double> state_vector;
        int r;
        int q;

    

    public:
    
        Estimate2d(int a, int b);

        //calculate state vector (zeta)
        arma::mat <double> updated_state_vector(turtlelib::Twist2D u);

        //calculate A matrix
        arma::mat <double> calculate_A_matrix(turtlelib::Twist2D u);

        //function to compute H
        arma::mat <double> calculate_h(arma::mat m_vec);

        //calculate r matrix
        arma::mat calculate_r_mat(int r);

        //calculate q matrix
        arma::mat calculate_q_mat(int q);

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
    
        //set r value
        void set_r(int a);

        //set q value
        void set_q(int a);

}

}












































#endif