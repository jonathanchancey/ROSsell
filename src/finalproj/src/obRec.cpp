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
#include <pcl_conversions/pcl_conversions.h> // TODO resolve duplicate of line 10?
#include <pcl_ros/transforms.h>
#include <nav_msgs/OccupancyGrid.h>
#include <sstream>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Quaternion.h>
#include <vector>
#include <map>
#include <iterator> 



using namespace ros;
using namespace std;


sensor_msgs::PointCloud2 ptCloud;
sensor_msgs::PointCloud2 ptCloudFiltered;
sensor_msgs::PointCloud2 ptCloudAux;
sensor_msgs::PointCloud filteredCloud;
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);

pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_map_cluster (new pcl::PointCloud<pcl::PointXYZ>);
sensor_msgs::PointCloud2 ptMapCloudFiltered;
map<double,double> objPoints;

vector<double> X;
vector<double> Y;
map<double, double>::iterator itr;

void mapConvert(const nav_msgs::OccupancyGrid::ConstPtr& msg){



  for(int width = 0; width < msg->info.width; ++width){
    for(int height = 0; height < msg->info.height; ++ height){
      if(msg->data[height*msg->info.width+width] > 0){
        X.push_back(width * msg->info.resolution+msg->info.resolution/2 -20);
        Y.push_back(height * msg->info.resolution + msg->info.resolution/2 -20);
      }
    }
  }

}




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

  // tf::TransformListener listener_;/




  LaserScanToPointCloud(ros::NodeHandle n) : 
    n_(n),
    laser_sub_(n_, "/scan", 10),
    laser_notifier_(laser_sub_,listener_, "base_link", 10)
  {
    laser_notifier_.registerCallback(
      boost::bind(&LaserScanToPointCloud::scanCallback, this, _1));
    laser_notifier_.setTolerance(ros::Duration(0.01));
    scan_pub_ = n_.advertise<sensor_msgs::PointCloud>("/my_cloud",1);
    scan_pub2_ = n_.advertise<sensor_msgs::PointCloud2>("/cloud_Ec",1);
    scan_pub3_ = n_.advertise<sensor_msgs::PointCloud2>("/cloud_noEc",1);
    scan_pub4_ = n_.advertise<sensor_msgs::PointCloud>("/fil_cloud",1);



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

    pcl::toROSMsg(*cloud_cluster,ptCloudFiltered);

    ptCloudFiltered.header.frame_id = "base_link";
    ptCloudFiltered.header.stamp = Time::now(); // problems of .20 seconds in the future

    // configures the map cloud in the saem way,
    // maybe needs to have map as frame id
    // not sure if time needs to be different
    ptMapCloudFiltered.header.frame_id = "map";
    ptMapCloudFiltered.header.stamp = Time::now(); // problems of .20 seconds in the future

    scan_pub2_.publish(ptCloudFiltered);
    
    sensor_msgs::convertPointCloud2ToPointCloud(ptCloudFiltered,filteredCloud); 
    // ran through eu filter, pcl to pointcloud2 to pointcloud1

    // TODO print content
    //ROS_INFO_STREAM("ptCloudFiltered.data" << cloud_cluster->points[0]);

    for(int c = 0; c < filteredCloud.points.size(); c++){
      for(int k = 0; k < X.size(); k++){
        if(filteredCloud.points[c].x != X[k] && filteredCloud.points[c].y != Y[k]){
          objPoints.insert(pair<double,double>((double)filteredCloud.points[c].x,(double)filteredCloud.points[c].y));
          //ROS_INFO_STREAM("Mailbox/Table found at " << filteredCloud.points[c].x << "," << filteredCloud.points[c].y << ")");

        }

      }
    }
    for(itr = objPoints.begin(); itr != objPoints.end(); ++itr){
      ROS_INFO_STREAM("Table/Mailbox found at (" << itr->first << "," << itr->second << ")");
    }

    // pcl::PointCloud<pcl::PointXYZ> cloud_cluster_;
    // pcl::fromROSMsg(cloud_cluster,cloud_cluster_);

    // TODO make new ptCloud for output
    // pcl_ros::transformPointCloud("map", ptCloudFiltered, ptMapCloudFiltered, listener_);
     

 

    scan_pub4_.publish(filteredCloud);

  }
  }
};



int main(int argc, char** argv)
{
  
  ros::init(argc, argv, "obRec");
  ros::NodeHandle n;
  LaserScanToPointCloud lstopc(n);
  Subscriber mapSub = n.subscribe("map", 1000, mapConvert);


  spin();

  
  return 0;

}
