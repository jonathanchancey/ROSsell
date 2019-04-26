#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <iostream>
#include <pcl_ros/point_cloud.h>
#include <pcl-1.7/pcl/point_cloud.h>

using namespace std;
using namespace ros;

void pclscanMessageReceived(const pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr&msg) {
  ROS_INFO_STREAM("I made it here");

}



void scanMessageReceived(const sensor_msgs::LaserScan&msg) {
  // ROS_INFO_STREAM("Received scan. Finding closest obstacle");
  float closest = msg.ranges[0];
  int closestIndex = 0;
  double count = 0;
  // string frameID = msg.header.frame_id;
  // for (int c = 500 ; c < 581 ; c++ ) {

  //     ROS_INFO_STREAM("distance is " << msg.ranges[c] << "at angle " << -10 + count);
  //     count += 0.25;
  // }
  // shutdown();
  // ROS_INFO_STREAM("Closest obstacle at distance (m)" << closest);
  // ROS_INFO_STREAM("Index: " << closestIndex);
  // ROS_INFO_STREAM("angle max " << msg.angle_max);
  // ROS_INFO_STREAM("Range min " << msg.angle_min);
}

void ptscanMessageReceived(const sensor_msgs::PointCloudConstPtr&msg) {
  ROS_INFO_STREAM("I made it here");

for(int c = 0; c < msg->points.size(); c++){
  ROS_INFO_STREAM(msg->points[c].x << " " << msg->points[c].y << " " << msg->points[c].z);


}

ROS_INFO_STREAM(msg->channels[0].name);

for(int c = 0; c < msg->channels.size(); c++){
  ROS_INFO_STREAM(msg->channels[0].values[0]);
}

}

void pt2scanMessageReceived(const sensor_msgs::PointCloudConstPtr&msg) {
  ROS_INFO_STREAM("I made it here");

for(int c = 0; c < msg->points.size(); c++){
  ROS_INFO_STREAM(msg->points[c].x << " " << msg->points[c].y << " " << msg->points[c].z);


}

ROS_INFO_STREAM(msg->channels[0].name);

for(int c = 0; c < msg->channels.size(); c++){
  ROS_INFO_STREAM(msg->channels[0].values[0]);
}

}





int main(int argc,char ** argv) {
  ros::init(argc,argv,"testNode");
  ros::NodeHandle nh;
  string topic = nh.resolveName("point_cloud");
  uint32_t queue_size  = 1;
  ros::Subscriber subScan = nh.subscribe("/scan",1000,&scanMessageReceived);
  ros::Subscriber ptScan = nh.subscribe("/scanPt",1000,&ptscanMessageReceived);
  ros::Subscriber pt2Scan = nh.subscribe("/scanPt2",1000,&pt2scanMessageReceived);
  Subscriber pclSub = nh.subscribe<pcl::PointCloud<pcl::PointXYZRGB> > ("/pclboi", 5000, &pclscanMessageReceived);

  spin();

}
