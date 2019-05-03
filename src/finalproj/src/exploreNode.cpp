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
using namespace std;

struct Coords { int x; int y; }; // s.d is a flexible array member 
Coords navPoints[19];

int main(int argc, char**argv){
    init(argc,argv,"exploreNode");
    NodeHandle nh;

    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base",true);
    ROS_INFO_STREAM("Waiting for server to be available...");
    while (!ac.waitForServer()) {}
    ROS_INFO_STREAM("done!");

    move_base_msgs::MoveBaseGoal goal;

    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = Time::now();
    goal.target_pose.pose.orientation.w = 1;

    double pointArrayX[19];
    double pointArrayY[19];
    double pointAllocator = -9;

    for(int c = 0; c < 19; c++){
        pointArrayX[c] = pointAllocator;
        pointArrayY[c] = pointAllocator;
        pointAllocator++;
    }

    // Coords xy;



    navPoints[0].x = 0;
    navPoints[0].y = 1;

    cout << "navPoints[0].y = " << navPoints[0].y << endl;


    Rate rate(1);



    while(ok()){
        Duration(1).sleep();
        // traveled array storing poitns of 

        // detect if current location has been travelled to 

        for(int x = 0;x < 19; x++){ //goes from (-9,-9) thru all the y's then up an x
            goal.target_pose.pose.position.x = pointArrayX[x];
            // cout << "(" << pointArrayX[x];
            for(int y = 0; y < 19; y++){
                goal.target_pose.pose.position.y = pointArrayY[y];
                // cout << "," << pointArrayY[y] << ")" << endl;
                ROS_INFO_STREAM(goal);
                ac.sendGoal(goal);
                ac.waitForResult(Duration(20));
                if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
                    spinOnce();
                    ROS_INFO_STREAM("Success");
                }
            }
            ROS_INFO_STREAM("All points have been visited!");
        }
    }
}
