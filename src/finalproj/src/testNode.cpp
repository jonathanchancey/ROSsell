#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <std_msgs/Int8MultiArray.h>
#include <sstream>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>


using namespace ros;
using namespace std;
int k = 0;

void mapConvert(const nav_msgs::OccupancyGrid::ConstPtr& msg){
  int Map[msg->info.height][msg->info.width];
  for(int i = 0; i < msg->info.width; i++){
    for(int j = 0; j < msg->info.height; j++){
      Map[j][i] = (int)msg->data[j*msg->info.width+i];
        k++;
    }
  }
  for(int i = 0; i < msg->info.width; i++){
    for(int j = 0; j < msg->info.height; j++){
      if(Map[i][j] != -1 && Map[i][j] != 100){
        ROS_INFO_STREAM("(" << i << "," << j << "):" << Map[i][j]);
      }
    }
  }
}

int main (int argc, char** argv){
    ros::init(argc, argv, "testNode");

    ros::NodeHandle n;

    Subscriber mapSub = n.subscribe("map", 1000, mapConvert);


    ros::spin();


  return (0);
}