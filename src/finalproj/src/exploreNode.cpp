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
#include <random>


using namespace ros;

int trueRandom(){
    std::random_device rd;
    std::mt19937 mt(rd());
    std::uniform_int_distribution<> dist(1,100);
    return dist(mt);
}

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



    while(ok()){
        Duration(2).sleep();
        goal.target_pose.pose.position.x = trueRandom() % 9 + -9;
        goal.target_pose.pose.position.y = trueRandom() % 9 + -9;
        ROS_INFO_STREAM(goal);
        ac.sendGoal(goal);
        ac.waitForResult(Duration(30));
        if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
            spinOnce();
            ROS_INFO_STREAM("Success");
        }


    }

}