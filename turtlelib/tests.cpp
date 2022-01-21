#define CATCH_CONFIG_MAIN
#include "catch.hpp"
#include <cmath>
#include <iostream>
#include "rigid2d.cpp"




TEST_CASE("Return a Identity Transformation"){

    turtlelib::Transform2D T;
    Vector2D v_test;




    v_test = T.translation()







    REQUIRE(v_test.y == 0);
    REQUIRE(v_test.y == 0);




}

