#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <std_msgs/Int8MultiArray.h>
#include <sstream>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>


using namespace ros;
using namespace std;
int j = 0;

void mapConvert(const nav_msgs::OccupancyGrid::ConstPtr& msg){
  for(int c = 0; c < msg->info.height*msg->info.width; c++){
    if((int)msg->data[c] != -1 && (int)msg->data[c] != 100){
      ROS_INFO_STREAM((int)msg->data.at(c));
      j++;

    }
    
  }
  ROS_INFO_STREAM(j);

}

int main (int argc, char** argv){
    ros::init(argc, argv, "testNode");

    ros::NodeHandle n;

    Subscriber mapSub = n.subscribe("map", 1000, mapConvert);


    ros::spin();


  return (0);
}