#include "ros/ros.h"
#include "tf/transform_listener.h"
#include "sensor_msgs/PointCloud.h"
#include "tf/message_filter.h"
#include "message_filters/subscriber.h"
#include "laser_geometry/laser_geometry.h"
#include "geometry_msgs/Point32.h"
#include <sensor_msgs/point_cloud_conversion.h>
#include <sensor_msgs/PointCloud2.h>


using namespace ros;
// #include <vector>
// #include <pcl_conversions/pcl_conversions.h> 
// #include <pcl_ros/transforms.h>

class LaserScanToPointCloud {
    public:

    ros::NodeHandle nh;

    laser_geometry::LaserProjection projector_;
    tf::TransformListener listener_;
    

    void scanCallback (const sensor_msgs::LaserScan::ConstPtr& scan_in)
    {
    if(!listener_.waitForTransform(
            scan_in->header.frame_id,
            "/base_link",
            scan_in->header.stamp + ros::Duration().fromSec(scan_in->ranges.size()*scan_in->time_increment),
            ros::Duration(1.0))){
        return;
    }

    sensor_msgs::PointCloud cloud;
    projector_.transformLaserScanToPointCloud("/base_link",*scan_in,
            cloud,listener_);

    // Do something with cloud.
    }
};


int main(int argc, char** argv)
{
  
  ros::init(argc, argv, "Rec");
  ros::NodeHandle n;
  LaserScanToPointCloud lstopc(n);
  Subscriber mapSub = n.subscribe("map", 1000, mapConvert);


  spin();

  
  return 0;

}