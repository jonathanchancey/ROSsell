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


using namespace ros;



int main(int argc, char**argv){
    init(argc,argv,"exploreNode");
    NodeHandle nh;

    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base",true);
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
            spinOnce();
            ROS_INFO_STREAM("Success");
        }


    }

}