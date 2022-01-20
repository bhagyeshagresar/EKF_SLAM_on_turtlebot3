#include <iostream>
#include "rigid2d.hpp"
#include <cmath>


int main(){
    
    turtlelib::Transform2D Tab, Tba, Tbc, Tcb, Tac, Tca;
    // turtlelib::Transform2D Tab_2(turtlelib::Vector2D v);
    // turtlelib::Transform2D Tab_3(turtlelib::deg2rad(theta));

    std::cout << "Enter transform T_ab" << std::endl;
   
    std::cin >> Tab;
    // std::cin >> turtlelib::Transform2D Tab_2(v);
    // std::cin >> turtlelib::Transform2D Tab_3(deg2theta);

    //Tab
    std::cout << "T_ab" << std::endl;
    std::cout << Tab << std::endl;


    //inverse Tba
    std::cout << "T_ba" << std::endl;
    Tba = Tab.inv();
    std::cout << Tba << std::endl;


    //Tbc
    std::cout << "T_bc" << std::endl;
    std::cout << Tbc << std::endl;


    //Tcb
    std::cout << "T_cb" << std::endl;
    Tcb = Tbc.inv();
    std::cout << Tcb << std::endl;
   

   //Tac
    std::cout << "T_cb" << std::endl;
    



    return 0;
}