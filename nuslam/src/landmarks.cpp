
#include <ros/ros.h>
#include <ros/console.h>
#include <vector>
#include "turtlelib/rigid2d.hpp"
#include "turtlelib/diff_drive.hpp"


std::vector <double> z; 


circle_fitting(std::vector <turtlelib::Vector2d> cluster, double n){
    
    //step 1 - compute the centroid
    
    double x_mean = (std::accumulate(cluster.x.begin(), cluster.x.end(), 0))/n;

    double y_mean = (std::accumulate(cluster.y.begin(), cluster.y.end(), 0))/n;

    //step 2 - shift the coordinates

    for (int i = 0; i < n; i++){
        double x_o_g = cluster.x.at(i) - x_mean;

        double y_o_g = cluster.y.at(i) - y_mean;

        //step 3
        z.at(i) = (std::pow(x_o_g, 2) + std::pow(y_o_g, 2));


    }

    //step 4 - compute mean
    double z_mean = (std::accumulate(z.begin(), z.end(), 0))/n;

    //step 8 - Compute H_inv

    arma::mat h_inv = {{0.0, 0.0, 0.0, 0.5},
                       {0.0, 1.0, 0.0, 0.0},
                       {0.0, 0.0, 1.0, 0.0},
                       {0.5, 0.0, 0.0, -2*z_mean}};


    







}