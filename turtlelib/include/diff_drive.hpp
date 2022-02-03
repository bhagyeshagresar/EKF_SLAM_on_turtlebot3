#include "rigid2d.hpp"


namespace turtlelib{

    struct Wheels
    {
        /// \brief right wheel pos
        double wheel_right = 0.0;

        /// \brief left wheel pos
        double wheel_left = 0.0;
    };


    struct Configuration
    {
        double x_config = 0.0;

        double y_config = 0.0;

        double theta_config = 0.0;

    }


    class DiffDrive
    {
        public:

        Wheels forward_kinematics(Wheels w);

    



        private:

        Wheels w;
        Configuration q;


    }


};