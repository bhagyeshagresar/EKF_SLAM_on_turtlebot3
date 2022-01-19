#include "rigid2d.hpp"
#include <cmath>



namespace turtlelib
{


    std::ostream & operator<<(std::ostream & os, const Vector2D & v){
        os << "[" << v.x << " " << v.y << "]" << endl;
        return os;
    }







    //Identity transformation
    Transform2D::Transform2D(){
        T_matrix[0][0] = cos(),
        T_matrix[[0][1] = 0,
        T_matrix[0][2] = 0,
        T_matrix[1][0] = 0,
        T_matrix[1][1] = 1,
        T_matrix[1][2] = 0,
        T_matrix[2][0] = 0,
        T_matrix[2][1] = 0,
        T_matrix[2][2] = 1,

        };
        
    }
    //Pure Translation
    Transform2D::Transform2D(Vector2D trans){
        T_matrix[0][0] = 0;
        T_matrix[[0][1] = 0;
        T_matrix[0][2] = trans.x;
        T_matrix[1][0] = 0;
        T_matrix[1][1] = 0;
        T_matrix[1][2] = trans.y;
        T_matrix[2][0] = 0;
        T_matrix[2][1] = 0;
        T_matrix[2][2] = trans.z;
    }

    //Pure rotation
    Transform2D::Transform2D(double radians){
        T_matrix[0][0] = cos(radians);
        T_matrix[[0][1] = -sin(radians);
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
        T_matrix[[0][1] = -sin(radians);
        T_matrix[0][2] = trans.x;
        T_matrix[1][0] = sin(radians);
        T_matrix[1][1] = cos(radians);
        T_matrix[1][2] = trans.y;
        T_matrix[2][0] = 0;
        T_matrix[2][1] = 0;
        T_matrix[2][2] = 1;
    
    }

    Vector2D Transform2D::operator()(Vector2D v) const{
        // double Trans_matrix[3][3]{
        //     {cos(radians)), -sin(rad2deg(radians)), trans.x},
        //     {sin(rad2deg(radians), cos(rad2deg(radians)), trans.y},
        //     {0, 0, 1}
        
        // };

        // double Trans_matrix[3][3]{
        //     {cos(radians), -sin(radians), x},
        //     {sin(radians), cos(radians), y},
        //     {0, 0, 1}

        // }
        double v_j[3] = {v.x, v.y, 1}; //pb

        Vector2D v_new[3]; //not sure
        v_new[3] = v_j;


    }
    
        for(int i = 0; i < 3; i++){
            for(int j = 0; j < 3; j++){
                int sum = 0;
                for (int k = 0; k < 3; k++){
                    sum += (T_matrix[i][k] * v_new[k])
                }
                v_new[i] = sum;
        }
        return v_new;
    }

    //  for(int i=0; i<r1; i++){
    //         for(int j=0; j<c2; j++){
    //             int sum =0;
    //             for(int k=0; k<r2; k++){
    //                 sum += (m1[i][k] * m2[k][j]);
    //             }
    //             res[i][j] = sum;
    //         }
    //     }
    
    Transform2D::inv() const{
            
        int inv_matrix[0][0] = T_matrix[0][0];
        int inv_matrix[0][1] = -T_matrix[0][1];
        int inv_matrix[0][2] = -trans.x*T_matrix[0][0] - trans.y*T_matrix[1][0];
        int inv_matrix[1][0] = T_matrix[0][1];
        int inv_matrix[1][1] = T_matrix[1][1];
        int inv_matrix[1][2] = -trans.y*T_matrix[0][0]+trans.y*T_matrix[0][1];
        int inv_matrix[2][0] = 0;
        int inv_matrix[2][1] = 0;
        int inv_matrix[2][2] = 1;

        return inv_matrix;

   }

    Transform2D::Transform2D & operator*=(const Transform2D & rhs){
        T_matrix[0][2] = T_matrix[0][2]*rhs[0][2];
        T_matrix[1][2] = T_matrix[1][2]*rhs[0][2];



        return *this;



    }


    Vector2D Transform2D::translation() const{

        trans_vector[0] = T_matrix[0][2];
        trans_vector[1] = T_matrix[1][2];
        trans_vector[2] = T_matrix[2][2];

        return trans_vector;

    }


    double Transform2D::rotation() const{
        rot_vector[0][0] = T_matrix[0][0];
        rot_vector[0][1] = T_matrix[0][1];
        rot_vector[0][0] = T_matrix[0][0];
        rot_vector[0][0] = T_matrix[0][0];




    }

//     Transform2D::Transform2D & operator*=(const Transform2D & rhs){
        
//     }

}


// Tac = Tab*Tbc
// Tab *= Tbc



























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




int main(){
    return 0;
}