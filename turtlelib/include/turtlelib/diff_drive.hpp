#include "turtlelib/rigid2d.hpp"


namespace turtlelib{

    struct Wheel_angles
    {
        /// \brief right wheel angle
        double w_ang1 = 0.0;

        /// \brief left wheel angle
        double w_ang2 = 0.0;
    };


    struct Configuration
    {
        double x_config = 0.0;

        double y_config = 0.0;

        double theta_config = 0.0;

    };


    struct Wheels_vel
    {
        double w1_vel = 0.0;
        double w2_vel = 0.0;
    };


    class DiffDrive
    {
        private:
        
        Wheel_angles w_ang;
        Wheels_vel w_vel;
        Configuration q;
        double d = 0.08;
        double r = 0.033;
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        
        public:

        DiffDrive();

        // DiffDrive(Wheel_angles w_ang, Wheels_vel w_vel, Configuration q);

        DiffDrive(Configuration q);


        Configuration forward_kinematics(Wheel_angles w_pos);

        Wheels_vel inverse_kinematics(Twist2D V);
        
        double get_radius();

        double get_length_d();




    



       


    };

    


}