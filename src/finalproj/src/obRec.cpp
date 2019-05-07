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
#include <pcl-1.7/pcl/kdtree/kdtree.h>
#include <pcl-1.7/pcl/common/common.h>
#include <pcl-1.7/pcl/common/projection_matrix.h>
#include <pcl-1.7/pcl/impl/point_types.hpp>
#include <pcl-1.7/pcl/ModelCoefficients.h>
#include <pcl-1.7/pcl/PolygonMesh.h>
#include <pcl-1.7/pcl/io/pcd_io.h>
#include <pcl-1.7/pcl/io/point_cloud_image_extractors.h>
#include <pcl-1.7/pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl-1.7/pcl/PointIndices.h>
#include <pcl_conversions/pcl_conversions.h>

#include <std_msgs/Int16.h>


using namespace ros;
using namespace std;

sensor_msgs::PointCloud2 ptCloud;
sensor_msgs::PointCloud2 ptCloudFiltered;
sensor_msgs::PointCloud2 ptCloudAux;
sensor_msgs::PointCloud filteredCloud;
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);

class LaserScanToPointCloud{

public:
  ros::NodeHandle n_;
  laser_geometry::LaserProjection projector_;
  tf::TransformListener listener_;
  message_filters::Subscriber<sensor_msgs::LaserScan> laser_sub_;
  tf::MessageFilter<sensor_msgs::LaserScan> laser_notifier_;
  ros::Publisher scan_pub_;
  ros::Publisher scan_pub2_;
  ros::Publisher scan_pub3_;
  ros::Publisher scan_pub4_;

  LaserScanToPointCloud(ros::NodeHandle n) : 
    n_(n),
    laser_sub_(n_, "/scan", 10),
    laser_notifier_(laser_sub_,listener_, "base_link", 10)
  {
    laser_notifier_.registerCallback(
      boost::bind(&LaserScanToPointCloud::scanCallback, this, _1));
    laser_notifier_.setTolerance(ros::Duration(0.01));
    scan_pub_ = n_.advertise<sensor_msgs::PointCloud>("/my_cloud",1);
    scan_pub2_ = n_.advertise<sensor_msgs::PointCloud2>("/cloud_Ec",1);// Ec is the euclidian filter
    scan_pub3_ = n_.advertise<sensor_msgs::PointCloud2>("/cloud_noEc",1);
    scan_pub4_ = n_.advertise<sensor_msgs::PointCloud>("/fil_cloud",1); // filter but as a pointcloud
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
        //std::cout << e.what();
        return;
    }
    
    // Do something with cloud.
    sensor_msgs::convertPointCloudToPointCloud2(cloud, ptCloud);
    //pcl::extractEuclideanClusters ec();

    pcl::VoxelGrid<pcl::PointXYZ> vg;
    pcl::PointCloud<pcl::PointXYZ>::Ptr pclCloud (new pcl::PointCloud<pcl::PointXYZ>), cloud_f (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(ptCloud,*pclCloud);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

    vg.setInputCloud (pclCloud);
    vg.setLeafSize (0.03f, 0.03f, 0.03f); //originally .01
    vg.filter (*cloud_filtered);
    //std::cout << "PointCloud after filtering has: " << cloud_filtered->points.size ()  << " data points." << std::endl;

    // Create the segmentation object for the planar model and set all the parameters
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ> ());
    seg.setOptimizeCoefficients (true); //true
    seg.setModelType (pcl::SACMODEL_LINE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setMaxIterations (100);
    seg.setDistanceThreshold (0.01); //0.01 play with me

    int i=0, nr_points = (int) cloud_filtered->points.size ();
    while (cloud_filtered->points.size () > 0.1 * nr_points) //0.3 play with me
    {
      // Segment the largest planar component from the remaining cloud
      seg.setInputCloud (cloud_filtered);
      seg.segment (*inliers, *coefficients);
      if (inliers->indices.size () == 0)
      {
        //std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
        break;
      }

      // Extract the planar inliers from the input cloud
      pcl::ExtractIndices<pcl::PointXYZ> extract;
      extract.setInputCloud (cloud_filtered);
      extract.setIndices (inliers);
      extract.setNegative (false);
      
      // Get the points associated with the linear surface
      extract.filter (*cloud_plane);
      //std::cout << "PointCloud representing the linear component: " << cloud_plane->points.size () << " data points." << std::endl;

      // Remove the linear inliers, extract the rest
      extract.setNegative (true);
      extract.filter (*cloud_f);
      *cloud_filtered = *cloud_f;
    }

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud (cloud_filtered);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance (0.01); // 2cm play with me
    ec.setMinClusterSize (1); //play with me
    ec.setMaxClusterSize (200); //play with me
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud_filtered);
    ec.extract (cluster_indices);

    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {
      for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
      cloud_cluster->points.push_back (cloud_filtered->points[*pit]); //*
      cloud_cluster->width = cloud_cluster->points.size ();
      cloud_cluster->height = 1;
      cloud_cluster->is_dense = true;
      //pcl_conversions::toPCL(Time::now(),cloud_cluster->header.stamp);
      //std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;

      pcl::toROSMsg(*cloud_cluster,ptCloudFiltered);

      ptCloudFiltered.header.frame_id = "base_link";
      ptCloudFiltered.header.stamp = Time::now();
      //ROS_INFO_STREAM(ptCloudFiltered);

      scan_pub2_.publish(ptCloudFiltered);

      sensor_msgs::convertPointCloud2ToPointCloud(ptCloudFiltered,filteredCloud);

      scan_pub4_.publish(filteredCloud);
    }
    pcl::toROSMsg(*cloud_filtered,ptCloudAux);
    scan_pub3_.publish(ptCloudAux);
    scan_pub_.publish(cloud);
    //scan_pub2_.publish(ptCloudFiltered);

    // for(int c = 0; c < ptCloudFiltered.data.size(); c++){
    //   cout << "point #" << c << " is: " << ptCloudFiltered << endl;
    // }
  }
};

void scanTriggerCallback(const std_msgs::Int16::ConstPtr& msg)
{
  ROS_INFO("Received value: %d \t spinnOnce() called",msg->data); // 
  // spinOnce();
  // ROS_INFO("I heard: [%d]", msg->);
}

int main(int argc, char** argv)
{
  
  ros::init(argc, argv, "obRec");
  ros::NodeHandle n;

  LaserScanToPointCloud lstopc(n);
  ros::Subscriber scan_trigger_sub = n.subscribe("/scanTrigger", 1000, scanTriggerCallback);


  // TODO reimplement spin function that spins when told from exploreNode, 
  spin();
  return 0;
}
