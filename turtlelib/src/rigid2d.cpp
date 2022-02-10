#include "turtlelib/rigid2d.hpp"
#include <cmath>
#include <iostream>



namespace turtlelib
{
    //vector

    std::ostream & operator<<(std::ostream & os, const Vector2D & v){

        os << "[" << v.x << " " << v.y << "]";
        return os;
    }


    std::istream & operator>>(std::istream & is, Vector2D & v){
        char c;
        c = is.peek();

        if (c = '['){
            is.get();
        }        
        is >> v.x;
        is >> v.y;
        
        char c1;
        c1 = is.peek();

        if (c1 = ']'){
            is.get();
        }


        return is;
    }

    Vector2D unit_vector(Vector2D v){
        Vector2D unit_v;

        unit_v.x = v.x/sqrt(v.x + v.y);
        unit_v.y = v.y/sqrt(v.x + v.y);

        return unit_v;
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
        
        double theta_inv {0.0};

        Vector2D invT;

        invT.x = -(v2.x*cos(theta))-(v2.y*sin(theta));
        invT.y = -(v2.y*cos(theta))+(v2.x*sin(theta));
        theta_inv = -theta;
       
        Transform2D T_inv{{invT.x, invT.y}, theta_inv};

        return T_inv;
   }

    Transform2D & Transform2D::operator*=(const Transform2D & rhs){
        Vector2D rhs_v;
        rhs_v.x = rhs.v2.x;
        rhs_v.y = rhs.v2.y;

        // theta = theta+rhs.theta;
        v2.x = v2.x + rhs_v.x*cos(theta)-sin(theta)*rhs_v.y;
        v2.y = v2.y + rhs_v.x*sin(theta)+cos(theta)*rhs_v.y;
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


    Twist2D Transform2D::new_twist(Twist2D V){
        Twist2D V_new;
        
        V_new.theta_dot = V.theta_dot;
        V_new.x_dot = v2.y*V.theta_dot + V.x_dot*cos(theta) -V.y_dot*sin(theta);
        V_new.y_dot = -v2.x*V.theta_dot + V.x_dot*sin(theta) + V.y_dot*cos(theta);

        return V_new;
    }






    std::istream & operator>>(std::istream & is, Transform2D & tf){
        std::string s1, s2, s3;
        
        
        is >> s1 >> tf.theta; 
        tf.theta = turtlelib::deg2rad(tf.theta);
        is >> s2 >> tf.v2.x;
        is >> s3 >> tf.v2.y;
        return is;
    }

    std::ostream & operator<<(std::ostream & os, const Transform2D & tf){
        // double 
        double theta = turtlelib::rad2deg(tf.theta);
        // double theta = tf.theta;
        os << theta <<" x: "<< tf.v2.x << " y: " << tf.v2.y;
        return os;
    }


    //twist 2d input
    std::istream &operator>>(std::istream &is, Twist2D &t){
        char c;
        c = is.peek();

         if (c = '['){
            is.get();
        }        
        is >> t.theta_dot;
        is >> t.x_dot;
        is >> t.y_dot;
       
        char c1;
        c1 = is.peek();

        if (c1 = ']'){
            is.get();
        }
        
        return is;

    }


    //twist 2d output
    std::ostream &operator<<(std::ostream &os, const Twist2D &t){
        os << "[" << t.theta_dot << " " << t.x_dot << " " << t.y_dot << "]";

    return os;
    }


    Transform2D integrate_twist(Twist2D V){
        double theta {0.0};


        if (V.theta_dot == 0.0){
            return Transform2D{Vector2D{V.x_dot, V.y_dot}};
        }
        
        else if (V.x_dot == 0.0 && V.y_dot == 0.0){
            return Transform2D{Vector2D{V.theta_dot}};

        
        }

        else{
            Vector2D v;
            v.x = V.y_dot/V.theta_dot;
            v.y = -V.x_dot/V.theta_dot;
            theta = V.theta_dot;

            Transform2D Tsb{{v.x, v.y}, theta};
            Transform2D Tss_{{0.0, 0.0}, theta};
            Transform2D Tbs = Tsb.inv();

            Transform2D Tbb_= Tbs*Tss_*Tsb;

            return Tbb_;
            

         }

    }

    

    


}






// output values in the twist2d object
// std::ostream &operator<<(std::ostream &os, const Twist2D &t){
//     os << t.twist_j;

//     return os;
// }

// insert values in the twist2d object  j
   


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




