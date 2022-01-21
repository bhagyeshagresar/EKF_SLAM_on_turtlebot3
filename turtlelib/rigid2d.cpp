#include "rigid2d.hpp"
#include <cmath>
#include <iostream>



namespace turtlelib
{
    //vector

    std::ostream & operator<<(std::ostream & os, const Vector2D & v){
        os << "[" << v.x << " " << v.y << "]" << std::endl;
        return os;
    }


    std::istream & operator>>(std::istream & is, Vector2D & v){
        is>> v.x >> v.y;
        return is;
    }

    

    //Identity transformation
    Transform2D::Transform2D()
        :v2{0, 0}, theta{0}
    
    {
       
    }
        
    
    //Pure Translation
    Transform2D::Transform2D(Vector2D trans)
        :v2{trans.x, trans.y}, theta{0}
    {
        
      
    }

    //Pure rotation
    Transform2D::Transform2D(double radians)
        :v2{0, 0}, theta{radians}
    {

    }

    //rotation and translation
    Transform2D::Transform2D(Vector2D trans, double radians)
        :theta{radians}, v2{v2.x, v2.y}{
    
    }

    Vector2D Transform2D::operator()(Vector2D v) const{
       
        // double theta = theta;
        // Vector2D v2 = v;
        Vector2D v_new;
        // double theta;


        v_new.x = (cos(theta)*v.x) - (sin(theta)*v.y) + v2.x;
        v_new.y = (sin(theta)*v.x) + (cos(theta)*v.y) + v2.y;


        return v_new;
    }

    
    Transform2D Transform2D::inv() const{   
        
       

        Transform2D invT;

        invT.theta = -theta;
        invT.v2.x = -(v2.x*cos(theta))-(v2.y*sin(theta));
        invT.v2.y = -(v2.y*cos(theta))+(v2.x*sin(theta));
        return invT;

   }

    Transform2D & Transform2D::operator*=(const Transform2D & rhs){
        std::cout << theta << std::endl;
        Transform2D rhs_v;
        rhs_v.v2.x = rhs.v2.x;
        rhs_v.v2.y = rhs.v2.y;

        // theta = theta+rhs.theta;
        v2.x = v2.x + rhs_v.v2.x*cos(theta)-sin(theta)*rhs_v.v2.y;
        v2.y = v2.y + rhs_v.v2.x*sin(theta)+cos(theta)*rhs_v.v2.y;
        theta = theta+rhs.theta;

        return *this;




    }


    Vector2D Transform2D::translation() const{
        Vector2D v3;

        v3.x = v2.x;
        v3.y = v2.y;


        return v3;

    }


    double Transform2D::rotation() const{
        double theta_2;

        theta_2 = theta;


        return theta_2;
    }

    Transform2D operator*(Transform2D lhs, const Transform2D & rhs){
        Transform2D tf_star;
        tf_star = lhs;
        tf_star *= rhs;
        return tf_star;
    }




    std::istream & operator>>(std::istream & is, Transform2D & tf){
        is >> tf.theta; 
        tf.theta = turtlelib::deg2rad(tf.theta);
        is >> tf.v2.x;
        is >> tf.v2.y;
        return is;
    }

    std::ostream & operator<<(std::ostream & os, const Transform2D & tf){
        // double 
        double theta = turtlelib::rad2deg(tf.theta);
        // double theta = tf.theta;
        os << theta <<" x: "<< tf.v2.x << " y: " << tf.v2.y;
        return os;
    }




}

























// default constructor for Twist2D class
// Twist2d::Twist2d()
//     :theta_dot{0}, dx{0}, dy{0}, theta{0}, x{0}, y{0}{
        

//     }



// output values in the twist2d object
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




