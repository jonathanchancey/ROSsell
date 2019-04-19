#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <math.h>

using namespace ros;



int main(int argc, char**argv){
    init(argc,argv,"moverobot");
    NodeHandle nh;

    Publisher movePub = nh.advertise<geometry_msgs::Twist>("/pioneer/cmd_vel", 1000);

    Rate rate(1);
    geometry_msgs::Twist cmd;



    while(ok()){
        cmd.linear.x = 1;
        cmd.angular.z = 0;
        movePub.publish(cmd);
        Duration(1).sleep();
        
        cmd.linear.x = 0;
        cmd.angular.z = M_PI/2;
        movePub.publish(cmd);
        Duration(1).sleep();

    }

}
