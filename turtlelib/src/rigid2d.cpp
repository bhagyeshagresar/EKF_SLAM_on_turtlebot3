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
        :v2{0, 0}, theta{0.0}
    
    {
       
    }
        
    
    //Pure Translation
    Transform2D::Transform2D(Vector2D trans)
        :v2{trans.x, trans.y}, theta{0.0}
    {
        
      
    }

    //Pure rotation
    Transform2D::Transform2D(double radians)
        :v2{0.0, 0.0}, theta{radians}
    {

    }

    //rotation and translation
    Transform2D::Transform2D(Vector2D trans, double radians)
        :v2{trans.x, trans.y}, theta{radians}{
    
    }

    Vector2D Transform2D::operator()(Vector2D v) const{
       
       
        Vector2D v_new;


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
        Vector2D v;

        if (V.theta_dot == 0.0){
            return Transform2D{Vector2D{V.x_dot, V.y_dot}};
        }
        
       
        
        else{
            v.x = V.y_dot/V.theta_dot;
            v.y = -V.x_dot/V.theta_dot;
            double theta = V.theta_dot;

            
            Transform2D Tsb{{v.x, v.y}, 0.0};
            Transform2D Tss_{{0.0, 0.0}, theta};
            Transform2D Tbs = Tsb.inv();

            Transform2D Tbb_= Tbs*Tss_*Tsb; 
            return Tbb_;
           
           
           
            

         }

       

    }

    //normalize angle
    double normalize_angle(double theta){
        double theta_3 = theta;
        while(theta_3 > M_PI){
            theta_3 -= 2*M_PI;
        }
        while(theta_3 < -M_PI){
            theta_3 += 2*M_PI;
        }
    
        return theta_3;

    }

    

 


}










