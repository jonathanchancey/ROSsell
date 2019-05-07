#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <math.h>
#include <stdlib.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Quaternion.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <std_msgs/Int16.h>

using namespace ros;

std_msgs::Int16 msg;


int main(int argc, char**argv){
    init(argc,argv,"exploreNode");
    NodeHandle nh;

    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base",true);
    ros::Publisher scan_trigger = nh.advertise<std_msgs::Int16>("/scanTrigger", 1);
    ROS_INFO_STREAM("Waiting for server to be available...");
    while (!ac.waitForServer()) {
    }
    ROS_INFO_STREAM("done!");

    move_base_msgs::MoveBaseGoal goal;

    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = Time::now();
    goal.target_pose.pose.orientation.w = 1;

    Rate rate(1);

    // fire on on signal from exploreNode, less walking on exploreNOde. It does random and then it turns a direction and then it scans, it turns north scans, west ,scans, south, scans, and then it can move again. 
    while(ok()){
        Duration(2).sleep();
        
        goal.target_pose.pose.position.x = rand() % 9 + -9;
        goal.target_pose.pose.position.y = rand() % 9 + -9;

        ROS_INFO_STREAM(goal);
        ac.sendGoal(goal);
        ac.waitForResult(Duration(30));
        
        if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
            // TODO send message to obRec that movement is complete. obRec will then scan.

            for(int i = 1; i <= 4;i++){

                //spin and scan?
                goal.target_pose.pose.orientation.w = i;
                ac.sendGoal(goal);
                //publishes 0 until new point SUCCEEDED
                msg.data = 0;
                scan_trigger.publish(msg);
                ac.waitForResult(Duration(10));
                
                if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
                    spinOnce();
                    msg.data = 1;
                    scan_trigger.publish(msg); // TODO see if works, it publishes a Int16. Maybe turn to a 0 immediately
                    // msg.data = 0;
                    // scan_trigger.publish(msg);
            }
            
            ROS_INFO_STREAM("Success");
        }
    }
}