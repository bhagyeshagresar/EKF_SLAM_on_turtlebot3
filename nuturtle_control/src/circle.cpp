#include <ros/ros.h>
#include <geometry_msgs/Twist.h>



static geometry_msgs::Twist twist;











int main(int argc, char **argv){
    
    ros::init(argc, argv, "circle");
    
    ros::NodeHandle nh;


    //Publish cmd_vel
    ros::Publisher cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 100);
    
    ros::ServiceServer control_service = nh.advertiseService("control", control_fn);

    ros::ServiceServer reverse_service = nh.advertiseService("reverse", reverse_fn);
    
    ros::ServiceServer stop_service = nh.advertiseService("stop", stop_fn);




    ros::Rate r(100);



    while(ros::ok()){
        
        
        ros::spinOnce();

        



        
        
        twist.linear = r*w;
        twist.angular = 0;




        




        r.sleep();

    }





}