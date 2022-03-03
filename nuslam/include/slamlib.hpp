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
        arma::mat <double> q;
        arma::mat <double> state_vector;
        int r;

    }

    public:

        Estimate2d();




}












































#endif