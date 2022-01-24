#include <iostream>
#include "rigid2d.hpp"
#include <cmath>


int main(){
    
    turtlelib::Transform2D Tab, Tba, Tbc, Tcb, Tac, Tca;
    turtlelib::Vector2D vb, va, vc, v_bhat;
    turtlelib::Twist2D V_b, V_a, V_c;
    // turtlelib::Transform2D Tab_2(turtlelib::Vector2D v);
    // turtlelib::Transform2D Tab_3(turtlelib::deg2rad(theta));

    std::cout << "Enter transform T_{a,b}:" << std::endl;
   
    std::cin >> Tab;
    // std::cin >> turtlelib::Transform2D Tab_2(v);
    // std::cin >> turtlelib::Transform2D Tab_3(deg2theta);

    std::cout << "Enter transform T_{b,c}:" << std::endl;

    std::cin >> Tbc;

    std::cout <<"T_{a, b}: deg: " << Tab << std::endl;


    //inverse Tba
    Tba = Tab.inv();
    std::cout <<"T_{b, a}: deg: " << Tba << std::endl;



    //Tbc
    std::cout <<"T_{b, c}: deg: " << Tbc << std::endl;



    //Tcb
    Tcb = Tbc.inv();
    std::cout <<"T_{c, b}: deg: " << Tcb << std::endl;


//    //Tac
    Tac = Tab*Tbc;
    std::cout <<"T_{a, c}: deg: " << Tac << std::endl;



//     //Tca
    Tca = Tac.inv();
    std::cout <<"T_{c, a}: deg: " << Tca << std::endl;


    
    std::cout << "Enter vector v_b:" << std::endl;
    std::cin >> vb;

    //va
    v_bhat = unit_vector(vb);
    va = Tab(vb);
    vb = Tba(va);
    vc = Tcb(vb);

    std::cout << "v_bhat: " << v_bhat << std::endl;
    std::cout << "v_a: " << va << std::endl;
    std::cout << "v_b: " << vb << std::endl;
    std::cout << "v_c: " << vc << std::endl;

    std::cout << "Enter Twist Vb" << std::endl;

    std::cin >> V_b;

    V_a = Tab.new_twist(V_b);
    V_b = Tba.new_twist(V_a);
    V_c = Tca.new_twist(V_a);
    std::cout <<"V_a " << V_a << std::endl;
    std::cout <<"V_b " << V_b << std::endl;
    std::cout <<"V_c " << V_c << std::endl;





    

    

    return 0;
}