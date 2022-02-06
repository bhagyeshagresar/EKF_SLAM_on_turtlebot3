// #define CATCH_CONFIG_MAIN
// #include "catch.hpp"
// #include <cmath>
// #include <iostream>
#include "turtlelib/rigid2d.hpp"
#include <catch_ros/catch.hpp>
#include "turtlelib/diff_drive.hpp"





TEST_CASE("Check translation function"){

    turtlelib::Transform2D T;
    turtlelib::Vector2D v_test;

    v_test = T.translation();




    REQUIRE(v_test.y == 0);
    REQUIRE(v_test.y == 0);
}



// TEST_CASE( "Test for inv", "[inverse]" ) {// Devesh Bhura

//     double theta = turtlelib::deg2rad(90);
//     turtlelib::Vector2D v, v0;
//     v.x = 0;
//     v.y = 0;
//     turtlelib::Transform2D Tab(v,theta), Tba;

//     v0 = Tab(v);


//     REQUIRE(turtlelib::almost_equal(v.x,v0.x,0.1));
//     REQUIRE(turtlelib::almost_equal(v.y,v0.y,0.1));
// }


// TEST_CASE( "Test for vector transformation", "Vector Transform" ) {// Devesh Bhura

//     turtlelib::Vector2D v, v0, vt;
//     double theta = turtlelib::deg2rad(90);
//     v.x = 1.0;
//     v.y = 1.0;
//     turtlelib::Transform2D Tab(v,theta);
//     vt.x = 0.0;
//     vt.y = 0.0;
//     v0 = Tab(vt);

//     REQUIRE(turtlelib::almost_equal(v0.x,v.x,0.01));
//     REQUIRE(turtlelib::almost_equal(v0.y,v.y,0.01));
// }

// // TEST_CASE( "Test for Twist transformation", "Twist Transform" ) {// Devesh Bhura

// //     turtlelib::Twist2D V0, Vt;
// //     turtlelib::Vector2D v;
// //     double theta = turtlelib::deg2rad(90);
// //     v.x = 1.0;
// //     v.y = 1.0;
// //     turtlelib::Transform2D Tab(v,theta);
// //     Vt.x_dot = 0.0;
// //     Vt.y_dot = 0.0;
// //     Vt.theta_dot = 0.0;
// //     V0 = Tab(Vt);

// //     REQUIRE(turtlelib::almost_equal(V0.x_dot,Vt.x_dot,0.01));
// //     REQUIRE(turtlelib::almost_equal(V0.y_dot,Vt.y_dot,0.01));
// //     REQUIRE(turtlelib::almost_equal(V0.theta_dot,Vt.theta_dot,0.01));
// // }


// TEST_CASE( "Test for Multiplication", "Multiplication" ) {// Devesh Bhura

//     turtlelib::Vector2D v, v0, vt;
//     double theta0;
//     double theta = turtlelib::deg2rad(90);
//     v.x = 1.0;
//     v.y = 1.0;
//     vt.x = 0.0;
//     vt.y = 0.0;
//     turtlelib::Transform2D Tab(v,theta), Tbc(vt,0);

//     Tab*=Tbc;

//     v0 = Tab.translation();
//     theta0 = Tab.rotation();


//     REQUIRE(turtlelib::almost_equal(theta0, theta,0.01));
//     REQUIRE(turtlelib::almost_equal(v0.x, v.x,0.01));
//     REQUIRE(turtlelib::almost_equal(v0.y, v.y,0.01));
// }

// TEST_CASE("test_case", "[some tag]")
// {
// 	REQUIRE (false);
// }


// TEST_CASE("test forward kinematics - forward"){
// 	turtlelib::DiffDrive D;
// 	turtlelib::Wheels_vel wheel;
// 	turtlelib::Twist2D V_fwd;
// 	// double r = 1.0;
// 	wheel.w1_vel = 3.0;
// 	wheel.w2_vel = 3.0;


// 	V_fwd = D.forward_kinematics(wheel);

// 	REQUIRE(V_fwd.x_dot == 3);
// 	REQUIRE(V_fwd.theta_dot == 0);




// }







// TEST_CASE("test forward kinematics - rotation"){
// 	turtlelib::DiffDrive D;
// 	turtlelib::Wheels_vel wheel;
// 	turtlelib::Twist2D V_fwd;
// 	// double r = 1.0;
// 	wheel.w1_vel = 3.0;
// 	wheel.w2_vel = -3.0;


// 	V_fwd = D.forward_kinematics(wheel);

// 	REQUIRE(V_fwd.x_dot == 0);
// 	REQUIRE(V_fwd.theta_dot == -1.5);




// }





// TEST_CASE("test inverse kinematics - forward"){
// 	turtlelib::DiffDrive D;
// 	turtlelib::Twist2D V_inv;
// 	turtlelib::Wheels_vel w_vel_test;
// 	V_inv.x_dot = 3.0;
// 	V_inv.theta_dot = 0.0;

// 	w_vel_test = D.inverse_kinematics(V_inv);

// 	REQUIRE(w_vel_test.w1_vel == 3);
// 	REQUIRE(w_vel_test.w2_vel == 3);




// }


// TEST_CASE("test inverse kinematics - rotation"){
// 	turtlelib::DiffDrive D;
// 	turtlelib::Twist2D V_inv;
// 	turtlelib::Wheels_vel w_vel_test;
// 	V_inv.x_dot = 0.0;
// 	V_inv.theta_dot = -1.5;

// 	w_vel_test = D.inverse_kinematics(V_inv);

// 	REQUIRE(w_vel_test.w1_vel == 3);
// 	REQUIRE(w_vel_test.w2_vel == -3);




// }