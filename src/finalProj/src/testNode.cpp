#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <iostream>

using namespace std;
using namespace ros;

void scanMessageReceived(const sensor_msgs::LaserScan&msg) {
  ROS_INFO_STREAM("Received scan. Finding closest obstacle");
  float closest = msg.ranges[0];
  int closestIndex = 0;
  // string frameID = msg.header.frame_id;
  for (int i = 1 ; i <msg.ranges.size() ; i++ ) {
    if (msg.ranges[i] < closest ) {
      closest = msg.ranges[i];
      closestIndex = i;
    }
  }
  ROS_INFO_STREAM("Closest obstacle at distance (m)" << closest);
  ROS_INFO_STREAM("Index: " << closestIndex);
  ROS_INFO_STREAM(msg.header.frame_id);
}

int main(int argc,char ** argv) {
  ros::init(argc,argv,"testNode");
  ros::NodeHandle nh;
  ros::Subscriber subScan = nh.subscribe("/scan",1000,&scanMessageReceived);
  ros::spin();
}
