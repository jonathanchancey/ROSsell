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
  for(int width = 0; width < msg->info.width; ++width){
    for(int height = 0; height < msg->info.height; ++ height){
      if(msg->data[height*msg->info.width+width] > 0){
        cout << "x: " << width * msg->info.resolution+msg->info.resolution/2 -20<< " ";
        cout << "y: " << height * msg->info.resolution + msg->info.resolution/2 -20<< endl;;
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