#include <ros/ros.h>
#include <std_msgs/UInt64.h>
#include <sensor_msgs/JointState.h>
#include <tf2/transform_broadcaster.h>
#include <string>
//service 


class SimNode
{
    
     private:
        ros::NodeHandle n("~");
        ros::Publisher timestep_pub;
        ros::Publisher js_pub;

        ros::ServiceServer reset_server;
        ros::ServiceServer teleport_server;

        ros::Subscriber transform_sub;
        std_msgs::UInt64 time_step;
        sensor_msgs::JointState red_joint_state;
        string joint_name;
        int time_step;
    
    public:
        SimNode():
            n{},
            reset_server(n.advertiseService("reset", reset_fn)),
            teleport_server(n.advertiseService("teleport", teleport_fn)),

            timestep_pub(n.advertise<std_msgs::UInt64>("time_step", 100)),
            js_pub(n.advertise<sensor_msgs::JointState>("red/joint_states", 100)),
            
            red_joint_state.name = "red_joint_states",
            red_joint_state.position.append(0),
            red_joint_state.velocity.append(0),
            
            transform_sub(n.subscribe("red_robot_pose", 10, &transfrom_callback))
            {
            }

        int reset_fn(&req){
            if (req.set){
                time_step = 0;
            }
            return time_step;
        }

        void teleport_fn(){
            //move to xyz location
        }
        
        void transform_callback(){ //need to figure out the arguments for pose
            static tf2::TransformBroadcaster br;
            tf2::Transform transform;
            transform.setOrigin(tf::Vector3(msg->x, msg->y, 0.0));
            tf2::Quaternion q;
            q.setRPY(0, 0, msg->theta);
            transform.setRotation(q);
            br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", "red_base_footprint"));
   
        }

        
        void main_loop(ros:Rate:Rate(500))
        {
            time_step = ROS::Time::now();
            timestep_pub.publish(time_step);
        }

    
   





};




int main(int argc, char **argv){

    ros::init(argc, argv, "nusim");
    SimNode sim;
    ros::spin();
    return 0;

}

