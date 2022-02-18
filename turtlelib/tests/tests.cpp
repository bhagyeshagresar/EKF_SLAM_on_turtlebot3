// #define CATCH_CONFIG_MAIN
// #include "catch.hpp"
#include <iostream>

#include "turtlelib/rigid2d.hpp"
#include <catch_ros/catch.hpp>
#include "turtlelib/diff_drive.hpp"
#include <cmath>






TEST_CASE("Check translation function", "[zero translation]"){

    turtlelib::Transform2D T;
    turtlelib::Vector2D v_test;

    v_test = T.translation();




    REQUIRE(v_test.x == 0);
    REQUIRE(v_test.y == 0);
}

TEST_CASE("Check translation function2", "[translation both positive]"){
    turtlelib::Vector2D v;
    v.x = 0.2;
    v.y = 0.3;
    turtlelib::Transform2D T{{v.x, v.y}, 0.0};
    turtlelib::Vector2D v_test;

    v_test = T.translation();




    REQUIRE(v_test.x == 0.2);
    REQUIRE(v_test.y == 0.3);
}



TEST_CASE("Check rotation function", "[zero rotation]"){
    turtlelib::Transform2D T;
    double theta_test;

    theta_test = T.rotation();

    REQUIRE(theta_test == 45);

}

TEST_CASE("Check rotation function2", "[positive rotation]"){
    double theta = turtlelib::deg2rad(45);
    double theta_test;
    turtlelib::Transform2D T{{0.0, 0.0}, theta};


    theta_test = T.rotation();

    REQUIRE(theta_test == turtlelib::deg2rad(45));

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

// TEST_CASE("Forward kinematics robot moving forward", "diffdrive"){

// }

// FWD Kinematics - Marco Morales

// TEST_CASE("Forward kinematics Robot driving forward", "[DiffDrive]") {
//     turtlelib::Configuration config;
//     turtlelib::Configuration new_config;
//     turtlelib::Wheel_angles phi;
//     turtlelib::Wheel_angles phi_new;
//     turtlelib::Wheel_angles phiold;
//     turtlelib::Wheels_vel phidot;
//     config.x_config = 0.0;
//     config.y_config = 0.0;
//     config.theta_config = 0.0;
//     phi.w_ang1 = 0.0;
//     phi.w_ang2 = 0.0;
//     phidot.w1_vel = 0.0;
//     phidot.w2_vel = 0.0;
   
    
//     turtlelib::DiffDrive D;



//     phi_new.w_ang1 = M_PI/4;
//     phi_new.w_ang2 = M_PI/4;

//     new_config = D.forward_kinematics(phi_new);
//     CHECK(new_config.x_config == Approx(0.033*(M_PI/4))); 
//     CHECK(new_config.y_config == Approx(0));
//     CHECK(new_config.theta_config == Approx(0)); 
// }

// TEST_CASE("Forward kinematics Robot driving rotation", "[DiffDrive]") {
//     turtlelib::Configuration config,new_config;
//     turtlelib::Wheel_angles phi,phi_new,phiold;
//     turtlelib::Wheels_vel phidot;
//     config.x_config = 0.0;
//     config.y_config = 0.0;
//     config.theta_config = 0.0;
//     phi.w_ang1 = 0.0;
//     phi.w_ang2 = 0.0;
//     phidot.w1_vel = 0.0;
//     phidot.w2_vel = 0.0;
   
//     turtlelib::DiffDrive D;

//     phi_new.w_ang1 = M_PI/4;
//     phi_new.w_ang2 = -M_PI/4;

//     new_config = D.forward_kinematics(phi_new);
//     CHECK(new_config.x_config == Approx(0.0)); 
//     CHECK(new_config.y_config == Approx(0));
//     CHECK(new_config.theta_config == Approx(-0.3239767424)); 
// }


// TEST_CASE("Forward kinematics Robot driving arc", "[DiffDrive]"){
//     turtlelib::Configuration config,new_config;
//     turtlelib::Wheel_angles phi,phi_new,phiold;
//     turtlelib::Wheels_vel phidot;
//     config.x_config = 0.0;
//     config.y_config = 0.0;
//     config.theta_config = 0.0;
//     phi.w_ang1 = 0.0;
//     phi.w_ang2 = 0.0;
//     phidot.w1_vel = 0.0;
//     phidot.w2_vel = 0.0;
   
//     turtlelib::DiffDrive D;

//     phi_new.w_ang1 = 19.992;
//     phi_new.w_ang2 = 27.6079;

//     new_config = D.forward_kinematics(phi_new);
//     CHECK(new_config.x_config == Approx(0.5)); 
//     CHECK(new_config.y_config == Approx(0.5));
//     CHECK(new_config.theta_config == Approx(M_PI/2)); 

// }


// TEST_CASE("Integrating a Twist", "[rigid2d]") { // Marco Morales & RKS
//     turtlelib::Twist2D twist;
//     turtlelib::Transform2D TbbPrime(0);
//     turtlelib::Vector2D trans;
//     double rot;
//     SECTION( "Testing Both Rotational and Translational components v2" ) {
//         twist.x_dot = 1.0;
//         twist.y_dot = 2.0;
//         twist.theta_dot = M_PI;
//         TbbPrime = integrate_twist(twist);
//         trans = TbbPrime.translation();
//         rot = TbbPrime.rotation();
//         CHECK( trans.x == Approx(-1.27324));
//         CHECK( trans.y ==  Approx(0.63662));
//         CHECK( rot ==  Approx(M_PI));
//     }

//     SECTION( "Testing Both Rotational and Translational components v3" ) {
//         twist.x_dot = 2.0;
//         twist.y_dot = 4.0;
//         twist.theta_dot = M_PI;
//         TbbPrime = integrate_twist(twist);
//         trans = TbbPrime.translation();
//         rot = TbbPrime.rotation();
//         CHECK( trans.x == Approx(-2.54).margin(0.01));
//         CHECK( trans.y ==  Approx(1.272).margin(0.01));
//         CHECK( rot ==  Approx(M_PI));
//     }

//     SECTION( "Testing Both Rotational and Translational components v4" ) {
//         twist.x_dot = 0.0;
//         twist.y_dot = 0.0;
//         twist.theta_dot = 0;
//         TbbPrime = integrate_twist(twist);
//         trans = TbbPrime.translation();
//         rot = TbbPrime.rotation();
//         CHECK( trans.x == Approx(0.0).margin(0.01));
//         CHECK( trans.y ==  Approx(0.0).margin(0.01));
//         CHECK( rot ==  Approx(0.0).margin(0.01));
//     }

//     SECTION( "Testing Both Rotational and Translational components v5" ) {
//         twist.x_dot = 5.0;
//         twist.y_dot = 2.0;
//         twist.theta_dot = 0.0;
//         TbbPrime = integrate_twist(twist);
//         trans = TbbPrime.translation();
//         rot = TbbPrime.rotation();
//         CHECK( trans.x == Approx(5.0).margin(0.01));
//         CHECK( trans.y ==  Approx(2.0).margin(0.01));
//         CHECK( rot ==  Approx(0).margin(0.01));
//     }

//     SECTION( "Testing Both Rotational and Translational components v6" ) {
//         twist.x_dot = 0.0;
//         twist.y_dot = 0.0;
//         twist.theta_dot = M_PI/4;
//         TbbPrime = integrate_twist(twist);
//         trans = TbbPrime.translation();
//         rot = TbbPrime.rotation();
//         CHECK( trans.x == Approx(0.0).margin(0.01));
//         CHECK( trans.y ==  Approx(0.0).margin(0.01));
//         CHECK( rot ==  Approx(M_PI/4));
//     }

//     SECTION( "Testing Both Rotational and Translational components v7" ) {
//         twist.x_dot = 5.0;
//         twist.y_dot = 2.0;
//         twist.theta_dot = M_PI/4;
//         TbbPrime = integrate_twist(twist);
//         trans = TbbPrime.translation();
//         rot = TbbPrime.rotation();
//         CHECK( trans.x == Approx(3.75574).margin(0.01));
//         CHECK( trans.y ==  Approx(3.66525).margin(0.01));
//         CHECK( rot ==  Approx(M_PI/4));
//     }
// }


// TEST_CASE("Check normalize function, theta = M_PI"){

//     turtlelib::Transform2D T_test;
//     double theta {M_PI};


//     theta = T_test.normalize_angle(theta);


//     REQUIRE(theta == M_PI);
// }




// TEST_CASE("Check normalize function, theta = -M_PI"){

//     turtlelib::Transform2D T_test;
//     double theta_test {-M_PI};

//     theta_test = T_test.normalize_angle(theta_test);


//     REQUIRE(theta_test == -M_PI);
// }

// TEST_CASE("Check normalize function, theta = 0.0"){

//     turtlelib::Transform2D T_test;
//     double theta_test {0.0};

//     theta_test = T_test.normalize_angle(theta_test);


//     REQUIRE(theta_test == 0.0);
// }

// TEST_CASE("Check normalize function, theta = -M_PI/4.0"){

//     turtlelib::Transform2D T_test;
//     double theta_test {-M_PI/4.0};

//     theta_test = T_test.normalize_angle(theta_test);


//     REQUIRE(theta_test == -M_PI/4.0);
// }

// TEST_CASE("Check normalize function, theta = -3M_PI/2"){

//     turtlelib::Transform2D T_test;
//     double theta_test {(3*M_PI)/2.0};

//     theta_test = T_test.normalize_angle(theta_test);


//     REQUIRE(theta_test == (3*M_PI)/2.0);
// }


// TEST_CASE("Check normalize function, theta = -5*M_PI/2.0"){

//     turtlelib::Transform2D T_test;
//     double theta_test {-(5*M_PI)/2.0};

//     theta_test = T_test.normalize_angle(theta_test);


//     REQUIRE(theta_test == -(5*M_PI)/2.0);
// }
