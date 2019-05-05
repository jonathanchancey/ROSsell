#include "ros/ros.h"
#include "tf/transform_listener.h"
#include "sensor_msgs/PointCloud.h"
#include "tf/message_filter.h"
#include "message_filters/subscriber.h"
#include "laser_geometry/laser_geometry.h"
#include "geometry_msgs/Point32.h"
#include <sensor_msgs/point_cloud_conversion.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl-1.7/pcl/point_cloud.h>
#include <pcl-1.7/pcl/segmentation/extract_clusters.h>
#include <pcl-1.7/pcl/kdtree/kdtree.h>
#include <pcl-1.7/pcl/common/common.h>
#include <pcl-1.7/pcl/common/projection_matrix.h>
#include <pcl-1.7/pcl/impl/point_types.hpp>
#include <pcl-1.7/pcl/ModelCoefficients.h>
#include <pcl-1.7/pcl/PolygonMesh.h>
#include <pcl-1.7/pcl/io/pcd_io.h>
#include <pcl-1.7/pcl/io/point_cloud_image_extractors.h>


using namespace ros;


sensor_msgs::PointCloud2 ptCloud;
pcl::PointCloud<pcl::PointXYZ> pclCloud;


class LaserScanToPointCloud{

public:

  ros::NodeHandle n_;
  laser_geometry::LaserProjection projector_;
  tf::TransformListener listener_;
  message_filters::Subscriber<sensor_msgs::LaserScan> laser_sub_;
  tf::MessageFilter<sensor_msgs::LaserScan> laser_notifier_;
  ros::Publisher scan_pub_;
    ros::Publisher scan_pub2_;



  LaserScanToPointCloud(ros::NodeHandle n) : 
    n_(n),
    laser_sub_(n_, "/scan", 10),
    laser_notifier_(laser_sub_,listener_, "base_link", 10)
  {
    laser_notifier_.registerCallback(
      boost::bind(&LaserScanToPointCloud::scanCallback, this, _1));
    laser_notifier_.setTolerance(ros::Duration(0.01));
    scan_pub_ = n_.advertise<sensor_msgs::PointCloud>("/my_cloud",1);
      scan_pub2_ = n_.advertise<sensor_msgs::PointCloud2>("/my_cloud2",1);

  }

  void scanCallback (const sensor_msgs::LaserScan::ConstPtr& scan_in)
  {
    sensor_msgs::PointCloud cloud;
    try
    {
        projector_.transformLaserScanToPointCloud(
          "base_link",*scan_in, cloud,listener_);
    }
    catch (tf::TransformException& e)
    {
        std::cout << e.what();
        return;
    }
    
    // Do something with cloud.
    sensor_msgs::convertPointCloudToPointCloud2(cloud, ptCloud);
    pcl::fromROSMsg(ptCloud,pclCloud);
    //pcl::extractEuclideanClusters ec();




    scan_pub_.publish(cloud);
    scan_pub2_.publish(ptCloud);

  }
};



int main(int argc, char** argv)
{
  
  ros::init(argc, argv, "obRec");
  ros::NodeHandle n;
  LaserScanToPointCloud lstopc(n);

  spin();

  
  return 0;

}
