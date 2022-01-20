#include <iostream>
#include "rigid2d.hpp"
#include <cmath>


int main(){
    
    turtlelib::Transform2D Tab, Tba, Tbc, Tcb, Tac, Tca;
    turtlelib::Vector2D vb, va, vc;
    // turtlelib::Transform2D Tab_2(turtlelib::Vector2D v);
    // turtlelib::Transform2D Tab_3(turtlelib::deg2rad(theta));

    std::cout << "Enter transform T_{a,b}:" << std::endl;
   
    std::cin >> Tab;
    // std::cin >> turtlelib::Transform2D Tab_2(v);
    // std::cin >> turtlelib::Transform2D Tab_3(deg2theta);

    std::cout << "Enter transform T_{b,c}:" << std::endl;

    std::cin >> Tbc;

    std::cout << Tab << std::endl;

    //inverse Tba
    Tba = Tab.inv();
    std::cout << Tba << std::endl;


    //Tbc
    std::cout << Tbc << std::endl;


    //Tcb
    Tcb = Tbc.inv();
    std::cout << Tcb << std::endl;
   

   //Tac
    Tac = Tab*Tbc;
    std::cout << Tac << std::endl;


    //Tca
    Tca = Tac.inv();
    std::cout << Tca << std::endl;

    
    std::cout << "Enter vector v_b" << std::endl;
    std::cin >> vb;

    //va

    std::cout << "Enter v_a" << std::endl;
    std::cout << Tab(vb) << std::endl;

    

    

    return 0;
}