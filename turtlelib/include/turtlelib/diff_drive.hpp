#include "turtlelib/rigid2d.hpp"


namespace turtlelib{

    /// \brief define a struct for the wheels
    struct Wheel_angles
    {
        /// \brief right wheel angle
        double w_ang1 = 0.0;

        /// \brief left wheel angle
        double w_ang2 = 0.0;
    };

    /// \brief define a struct for the configuration of the robot
    struct Configuration
    {
        double x_config = 0.0;

        double y_config = 0.0;

        double theta_config = 0.0;

    };

    /// \brief define a struct for the wheel velocities
    struct Wheels_vel
    {
        double w1_vel = 0.0;
        double w2_vel = 0.0;
    };

    /// \brief DiffDrive Class to compute the forward kinematics and inverse kinematics of the robot
    class DiffDrive
    {
        private:
        
        Wheel_angles w_ang;
        Wheels_vel w_vel;
        Configuration q;
        double d = 0.08; // distance between the centre of the chassis and the front or back of the choice
        double r = 0.033; //radius of the robot wheel
        
        
        
        
        public:

        /// \brief default constructor for DiffDrive Class
        DiffDrive();

        /// \brief constructor with arguments
        /// \param q - Configuration of the robot
        DiffDrive(Configuration q);

        /// \brief function to calculate the configuration of the robot given the wheel angles
        /// the function updates the wheel angles and the wheel velocities and then calculates the new twist
        /// Then we find the Transformation matrix of the robot wrt world using current configuration and then find the transformation matrix
        /// of the robot in new frame wrt old frame using the integrate twist function. Using the subscript cancellation rule we 
        /// get the transformation matrix of the robot in the new frame wrt world and then use translation and rotation function from
        /// Transform2D class to get the current configuration of the robot.
        /// \param w_pos - wheel angles
        /// \return q - Current Configuration of the robot
        Configuration forward_kinematics(Wheel_angles w_pos);


        /// \brief function to calculate the wheel velocities given the robot twist
        /// \param V - Twist2D
        /// \return wheel velocities
        Wheels_vel inverse_kinematics(Twist2D V);
        
        /// \brief function to get the radius of the wheels
        double get_radius();

        /// \brief function to get the length d
        double get_length_d();

        /// \brief function to get the configuration of the robot
        Configuration get_config();


        /// \brief function to set the configuration of the robot
        void set_config(Configuration new_q);




    
       


    };

    


}