#include "rigid2d.hpp"
#include <cmath>
#include <iostream>



namespace turtlelib
{


    std::ostream & operator<<(std::ostream & os, const Vector2D & v){
        os << "[" << v.x << " " << v.y << "]" << std::endl;
        return os;
    }


    std::istream & operator>>(std::istream & is, Vector2D & v){
        is>> v.x >> v.y;
        return is;
    }

    

    //Identity transformation
    Transform2D::Transform2D(){
        T_matrix[0][0] = cos(theta);
        T_matrix[0][1] = -sin(theta);
        T_matrix[0][2] = v.x;
        T_matrix[1][0] = sin(theta);
        T_matrix[1][1] = cos(theta);
        T_matrix[1][2] = v.y;
        T_matrix[2][0] = 0;
        T_matrix[2][1] = 0;
        T_matrix[2][2] = 1;


    }
        
    
    //Pure Translation
    Transform2D::Transform2D(Vector2D trans){
        T_matrix[0][0] = 0;
        T_matrix[0][1] = 0;
        T_matrix[0][2] = trans.x;
        T_matrix[1][0] = 0;
        T_matrix[1][1] = 0;
        T_matrix[1][2] = trans.y;
        T_matrix[2][0] = 0;
        T_matrix[2][1] = 0;
        T_matrix[2][2] = 1;
    }

    //Pure rotation
    Transform2D::Transform2D(double radians){
        T_matrix[0][0] = cos(radians);
        T_matrix[0][1] = -sin(radians);
        T_matrix[0][2] = 0;
        T_matrix[1][0] = sin(radians);
        T_matrix[1][1] = cos(radians);
        T_matrix[1][2] = 0;
        T_matrix[2][0] = 0;
        T_matrix[2][1] = 0;
        T_matrix[2][2] = 1;
    }

    //rotation and translation
    Transform2D::Transform2D(Vector2D trans, double radians){
        T_matrix[0][0] = cos(radians);
        T_matrix[0][1] = -sin(radians);
        T_matrix[0][2] = trans.x;
        T_matrix[1][0] = sin(radians);
        T_matrix[1][1] = cos(radians);
        T_matrix[1][2] = trans.y;
        T_matrix[2][0] = 0;
        T_matrix[2][1] = 0;
        T_matrix[2][2] = 1;
    
    }

    Vector2D Transform2D::operator()(Vector2D v) const{
       

        Vector2D v_new; 


        v_new.x = (cos(theta)*v.x) - (sin(theta)*v.y) + v.x;
        v_new.y = (sin(theta)*v.y) + (cos(theta)*v.y) + v.y;
        return v_new;
    }

    
    Transform2D Transform2D::inv() const{
        
        Transform2D invT;
        // invT.theta = 
        invT.T_matrix[0][0] = cos(theta);
        invT.T_matrix[0][1] = sin(theta);
        invT.T_matrix[0][2] = -(v.x*cos(theta)) - (v.y*sin(theta));
        invT.T_matrix[1][0] = -sin(theta);
        invT.T_matrix[1][1] = cos(theta);
        invT.T_matrix[1][2] = -(v.y*cos(theta))+(v.x*sin(theta));
        invT.T_matrix[2][0] = 0;
        invT.T_matrix[2][1] = 0;
        invT.T_matrix[2][2] = 1;

        invT.theta = asin(invT.T_matrix[1][0]);
        invT.v.x = invT.T_matrix[0][2];
        invT.v.y = invT.T_matrix[1][2];
        return invT;

   }

    Transform2D Transform2D:: & operator*=(const Transform2D & rhs){
        Transform2D T_matrix2;
        // Translational component
        T_matrix2[0][2] = T_matrix[0][2]*rhs[0][2];
        T_matrix2[1][2] = T_matrix[1][2]*rhs[0][2];
        T_matrix2[1][2] = T_matrix[1][2]*rhs[0][2];





        return *this;



    // }


    Vector2D Transform2D::translation() const{
        Vector2D v2;

        v2.x = T_matrix2[0][2];
        v2.y = T_matrix2[1][2];


        return v2;

    }


    double Transform2D::rotation() const{
        rot_vector[0][0] = T_matrix[0][0];
        rot_vector[0][1] = T_matrix[0][1];
        rot_vector[0][0] = T_matrix[0][0];
        rot_vector[0][0] = T_matrix[0][0];




    }



    std::istream & operator>>(std::istream & is, Transform2D & tf){
        is >> tf.theta; 
        tf.theta = turtlelib::deg2rad(tf.theta);
        is >> tf.v.x;
        is >> tf.v.y;
        // tf = turtlelib::Transform2D(tf.v, tf.theta);
        return is;
    }

    std::ostream & operator<<(std::ostream & os, const Transform2D & tf){
        double theta = turtlelib::rad2deg(tf.theta);
        // double theta = tf.theta;
        os << theta <<" x: "<< tf.v.x << " y: " << tf.v.y;
        return os;
    }




}

























// // default constructor for Twist2D class
// Twist2d::Twist2d()
//     :theta_dot{0}, dx{0}, dy{0}, theta{0}, x{0}, y{0}{
        

//     }



// // output values in the twist2d object
// std::ostream &operator<<(std::ostream &os, const Twist2d &t){
//     os << t.twist_j;

//     return os;
// }

// // insert values in the twist2d object  j
// std::istream &operator>>(std::istream &in, Twist2d &t){
//     cout << "enter values " << endl;
//     in >> t.thetadot;
//     in >> t.dx;
//     in >> t.dy;
//     in >> t.theta;
//     in >> t.x;
//     in >> t.y;

//     return in;

// }


// //function to return Twist vector in frame i
// double Twist2d::twist_frame_i(double theta_dot, double dx, double dy){
//     double twist_i[3] {theta_dot, dx, dy};

//     return twist_i;
// }

// //function to return adjoint
// double Twist2d::adjoint(double theta, double x, double y){
//     const int rows {3};
//     const int cols {3};
    
//     double adjoint[rows][cols]
//     {
//         {1, 0, 0},
//         {y, cos(theta), -sin(theta)},
//         {-x, sin(theta), cos(theta)}
//     };

//     return adjoint;
    
// }

// //function to return twist in frame j
// double twist_frame_j(double twist_i, double adjoint){
//     double twist_j = twist_i*adjoint;

//     return twist_j;
// }



