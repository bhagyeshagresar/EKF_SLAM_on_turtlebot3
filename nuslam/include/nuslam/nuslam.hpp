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
       
        arma::mat covariance;
        arma::mat q_mat;
        arma::mat r_mat;
        double r;
        double q;
        arma::mat m_vec;
        arma::mat predict_vector;
      
        
        
    

    public:
    
        Estimate2d(double r, double q);

        //calculate state vector (zeta)
        arma::mat updated_state_vector(turtlelib::Twist2D u, arma::mat prev_state_vector);

        //calculate A matrix
        arma::mat calculate_A_matrix(turtlelib::Twist2D u);

        //function to compute H
        arma::mat calculate_h(int i);

        

        //calculate z
        arma::mat calculate_z(double g, double y);

        //calculate z_hat
        arma::mat calculate_z_hat(int i);

        //get covariance
        arma::mat get_covariance();

       
        //get q_matrix
        arma::mat get_q_matrix();

        //get r_matrix
        arma::mat get_r_matrix();

        arma::mat init_fn(arma::mat temp_vec);

        

        void set_prev_vector(arma::mat a);

        void set_predict_vector(arma::mat final_vector);

        arma::mat get_predict_vector();

        void set_covariance(arma::mat c);
    
       
};

}



#endif
