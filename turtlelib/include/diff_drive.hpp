#include "rigid2d.hpp"


namespace turtlelib{

    struct Wheels_pos
    {
        /// \brief right wheel pos
        double w_1 = 0.0;

        /// \brief left wheel pos
        double w_2 = 0.0;
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
        public:

        DiffDrive();

        Twist2D forward_kinematics(Wheels_vel w);

        Wheels_vel inverse_kinematics(Twist2D V);





    



        private:

        Wheels_pos w;
        Wheels_vel w_vel;
        Configuration q;
        double d;
        double r;


    };


}