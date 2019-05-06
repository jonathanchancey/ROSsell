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
Coords traveledPoints[19];
geometry_msgs::Vector3 pos;
Coords currentLoc;

void Int16NextpointReceived(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& pmsg){
    pos.x = pmsg->pose.pose.position.x;
    pos.y = pmsg->pose.pose.position.y;
}

int main(int argc, char**argv){
    init(argc,argv,"exploreNode");
    NodeHandle nh;

    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base",true);
    ROS_INFO_STREAM("Waiting for server to be available...");
    while (!ac.waitForServer()) {}
    ROS_INFO_STREAM("done!");

    ros::Subscriber pose_sub = nh.subscribe("/amcl_pose", 1000, Int16NextpointReceived);

    move_base_msgs::MoveBaseGoal goal;

    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = Time::now();
    goal.target_pose.pose.orientation.w = 1;

    double pointArrayX[19];
    double pointArrayY[19];
    double pointAllocator = -9;

    for(int c = 0; c < 19; c++){
        navPoints[c].x = pointAllocator;
        navPoints[c].y = pointAllocator;
        pointAllocator++;
    }

    cout << "navPoints[0].y = " << navPoints[0].y << endl;


    Rate rate(1);


    while(ok()){
        Duration(1).sleep();
        // traveled array storing poitns of positions the robot has been
        
        ROS_INFO("pos.x = %f\t pos.y  = %f",pos.x,pos.y);

        currentLoc.x = pos.x;
        currentLoc.y = pos.y;

        // TODO fix magic number
        // checks if current location is in traveled location array
        for (int i = 0; i < 19; i++){
            
            if ((traveledPoints[i].x == currentLoc.x) && (traveledPoints[i].y == currentLoc.y)){
                // Wow this if statement should be inverted. 
                // TODO maybe do ^
            } else {
                printf("Adding %d,%d to traveledPoints", currentLoc.x, currentLoc.y);
                traveledPoints[i].x = currentLoc.x;
                traveledPoints[i].y = currentLoc.y;
            }

        }
        // bool exists = std::any_of(std::begin(navPoints), std::end(navPoints), [&](Coords i)
        // {
        //     return i == currentLoc;
        // });

        // if (nearestPoint(pos.x,pos.y)){

        // }

        // detect if current location has been travelled to 

        for(int x = 0;x < 19; x++){ //goes from (-9,-9) thru all the y's then up an x
            // cout << "(" << pointArrayX[x];
            for(int y = 0; y < 19; y++){
                // checks if next point has already been travelled
                for (int i = 0; i < 19; i++){
                    if ((traveledPoints[i].x == navPoints[x].x) && (traveledPoints[i].y == navPoints[y].y)){
                        // If this wasn't for sure 19x19 then this would get big(slOw)
                        printf("Skipping point %d,%d as was in traveledPoints", navPoints[x].x, navPoints[y].y);
                        goto ENDOF2NDFOR;
                    }
                }
                goal.target_pose.pose.position.x = navPoints[x].x;
                goal.target_pose.pose.position.y = navPoints[y].y;
                // cout << "," << pointArrayY[y] << ")" << endl;
                ROS_INFO_STREAM(goal);
                ac.sendGoal(goal);
                ac.waitForResult(Duration(20));
                if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
                    spinOnce();
                    ROS_INFO_STREAM("Success");
                }
                ENDOF2NDFOR:
            }
            ROS_INFO_STREAM("All points have been visited!");
        }
    }
}
