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
#include <std_msgs/Bool.h>

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

    Publisher pubGoal = nh.advertise<std_msgs::Bool>("/fireFire", 1000);
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base",true);
    ROS_INFO_STREAM("Waiting for server to be available...");

    while (!ac.waitForServer()) {
    }
    ROS_INFO_STREAM("done!");

    move_base_msgs::MoveBaseGoal goal;
    bool firstRun = true;

    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = Time::now();
    goal.target_pose.pose.orientation.w = 1;

    std_msgs::Bool sendIt;



    Rate rate(1);



    while(ok()){
        Duration(2).sleep();

        if(firstRun == true){
            goal.target_pose.pose.position.x = 0;
            goal.target_pose.pose.position.y = 0;
            ROS_INFO_STREAM(goal);
            ac.sendGoal(goal);
            ac.waitForResult();
            if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
            sendIt.data = true;
            pubGoal.publish(sendIt);
            Duration(5).sleep();
            sendIt.data = false;
            pubGoal.publish(sendIt);
            firstRun = false;
            spinOnce();
            }
        }

        goal.target_pose.pose.position.x = trueRandom() % 9 + -9;
        goal.target_pose.pose.position.y = trueRandom() % 9 + -9;
        ROS_INFO_STREAM(goal);
        ac.sendGoal(goal);
        ac.waitForResult(Duration(30));
        if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
            ROS_INFO_STREAM("Success");
            sendIt.data = true;
            pubGoal.publish(sendIt);
            Duration(5).sleep();
            sendIt.data = false;
            pubGoal.publish(sendIt);
            spinOnce();


        }


    }

}