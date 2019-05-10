#include "ros/ros.h"
#include "tf/transform_listener.h"
#include "sensor_msgs/PointCloud.h"
#include "tf/message_filter.h"
#include "message_filters/subscriber.h"
#include "laser_geometry/laser_geometry.h"
#include "geometry_msgs/Point32.h"
#include <sensor_msgs/point_cloud_conversion.h>
#include <sensor_msgs/PointCloud2.h>

#include <pcl_ros/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <nav_msgs/OccupancyGrid.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/extract_indices.h>
#include <vector>
#include <algorithm>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
// #include <map>
// #include <iterator>
// disregard last commit 

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
double currX = 0;
double currY = 0;
double currTheta = 0;
double yaw = 0;

struct Tables{
  double midX,midY;
};
struct MailBoxes{
  double midX,midY;

};
vector<Tables> tablesVec;
vector<MailBoxes> mailBoxVec;

void amclReceived(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg){

    currX = msg->pose.pose.position.x;
    currY = msg->pose.pose.position.y;
    currTheta = msg->pose.pose.orientation.w;
    yaw = atan2(2.0*(msg->pose.pose.orientation.y*msg->pose.pose.orientation.z + msg->pose.pose.orientation.w*msg->pose.pose.orientation.x),msg->pose.pose.orientation.w*msg->pose.pose.orientation.w - msg->pose.pose.orientation.x*msg->pose.pose.orientation.x - msg->pose.pose.orientation.y*msg->pose.pose.orientation.y + msg->pose.pose.orientation.z*msg->pose.pose.orientation.z);
    
  //  ROS_INFO_STREAM("this is angle" << yaw << " ");
  

}


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


// prints objects found so far
void printFoundObjects(){
  // ROS_INFO("Entered printFoundObjects()");

  ROS_INFO_STREAM("Found Objects List:");

  if (!tablesVec.empty()){
    for (int i = 0; i < tablesVec.size(); i++){
      ROS_INFO_STREAM("tables[" << i << "]" << " at position " << tablesVec[i].midX << "," << tablesVec[i].midY);
    }
  }

  if (!mailBoxVec.empty()){
    for (int i = 0; i < mailBoxVec.size(); i++){
      ROS_INFO_STREAM("mailbox[" << i << "]" << " at position " << mailBoxVec[i].midX << "," << mailBoxVec[i].midY);
    }
  }
}



struct compare
{
	double key;
	compare(double const &i): key(i) { }

	bool operator()(double const &i)
	{
		return (i == key);
	}
};

bool inRange(double low, double high, double x) 
{ 
  if((x-low) <= (high-low)){
    return true;
  }
  else{
    return false;
  }
}

bool tableMaybe(double x, double y, sensor_msgs::PointCloud* pt){
    // input x y is suspected table leg
    // searches all other points in cloud 
    // attempts to find table
    double thresholdWidth  = 1.8;
    double thresholdLength = 1.32;

    double x2 = 0;
    double y2 = 0;

    double xdiff = 0;
    double ydiff = 0;

    double xAligndiff = 0;
    double yAligndiff = 0;

    double xMidpoint = 0;
    double yMidpoint = 0;

    double xMidpointDiff = 0;
    double yMidpointDiff = 0;

    Tables table;

    // ROS_INFO_STREAM("tableMaybe called");

    double acceptablePointError = .02; // accounts for cloud distribution

    double acceptableCenterError = 1.0; // accounts for cloud distribution
    for(int i  = 0; i < pt->points.size(); i++){
        x2 = pt->points[i].x;
        y2 = pt->points[i].y;

        xdiff = fabs(x-x2);
        ydiff = fabs(y-y2);
        //ROS_INFO_STREAM(xdiff);
        //ROS_INFO_STREAM(ydiff);
        // if(inRange(thresholdWidth - acceptablePointError, thresholdWidth + acceptablePointError, xdiff)){
        //   if(inRange(thresholdLength - acceptablePointError, thresholdLength + acceptablePointError, ydiff)){
        if(xdiff > thresholdWidth - acceptablePointError && xdiff < thresholdWidth + acceptablePointError){
          if(ydiff > thresholdLength - acceptablePointError && ydiff < thresholdLength + acceptablePointError){
            if(!(find(X.begin(), X.end(), x2) != X.end())){
              if(!(find(Y.begin(), Y.end(), y2) != Y.end())){
              // find center by taking difference between 2 points -- midpoint formula
              xMidpoint = (x+x2)/2;
              yMidpoint = (y+y2)/2;
              // save in intervals of .5 coordinates?
              // remove entries that are within 1 coord away?
              //ROS_INFO_STREAM("Table at (" << x2 << "," << y2 << ")");
              //ROS_INFO_STREAM("Calculated midoint at " << xMidpoint << "," << yMidpoint);
              //TODO maybe needs to be shifted so center is output 

                //if(none_of(tablesVec.begin(),tablesVec.end(),this.x = placeholders::_1xMidpoint ))

                
               if(tablesVec.size() >= 1){// changed to >= from > because it starts at 0 
              //  ROS_INFO("currTheta = %f", currTheta); 
               
                for(int i = 0;i<tablesVec.size();i++){

                  xMidpointDiff = fabs((xMidpoint + currX)-tablesVec[i].midX); // added (xMidpoint + currX) 
                  yMidpointDiff = fabs((yMidpoint + currY)-tablesVec[i].midY);

                  // ROS_INFO("MidPointDiff = %f,%f",xMidpointDiff,yMidpointDiff);
                  if(xMidpointDiff > acceptableCenterError){
                    ROS_INFO_STREAM("Adding TABLE xmidmidpoint,ypoint" << xMidpoint + currX << "," << yMidpoint + currY);

                    table.midX = xMidpoint + currX;
                    table.midY = yMidpoint + currY;
                    tablesVec.push_back(table);
                    printFoundObjects();
                    return 0;
                  }
                }
               }
               else{
                  ROS_INFO_STREAM("Adding THE FIRST TABLE point xmidmidpoint,ypoint" << xMidpoint + currX << "," << yMidpoint + currY);

                    table.midX = xMidpoint + currX;
                    table.midY = yMidpoint + currY;
                    tablesVec.push_back(table);
                    printFoundObjects();
               }
                // ROS_INFO_STREAM(tablesVec.size());
              }
            }
          }
        }
    }
}


bool maybeMailbox(double x, double y, sensor_msgs::PointCloud* pt){
        // input x y is suspected table leg
    // searches all other points in cloud 
    // attempts to find table
    double thresholdWidth  = 0.51;
    double thresholdLength = 0.53;

    double x2 = 0;
    double y2 = 0;

    double xdiff = 0;
    double ydiff = 0;

    double xMidpoint = 0;
    double yMidpoint = 0;

    double xMidpointDiff = 0;
    double yMidpointDiff = 0;

    MailBoxes mb;
    double acceptablePointError = .02; // accounts for cloud distribution

    double acceptableCenterError = 1.0; // we know the name hasn't changed, this is for mailbox 
    for(int i  = 0; i < pt->points.size(); i++){
        x2 = pt->points[i].x;
        y2 = pt->points[i].y;

        xdiff = fabs(x-x2);
        ydiff = fabs(y-y2);
        if(xdiff > thresholdWidth - acceptablePointError && xdiff < thresholdWidth + acceptablePointError){
          if(ydiff > thresholdLength - acceptablePointError && ydiff < thresholdLength + acceptablePointError){
            if(!(find(X.begin(), X.end(), x2) != X.end())){
              if(!(find(Y.begin(), Y.end(), y2) != Y.end())){
                xMidpoint = (x+x2)/2;
                yMidpoint = (y+y2)/2;
                //ROS_INFO_STREAM("Mailbox at (" << x2 << "," << y2 << ")");
                //  ROS_INFO_STREAM("Calculated midoint at " << xmidpoint << "," << ymidpoint);



                  
               if(!mailBoxVec.empty()){// changed to >= from > because it starts at 0 
                  
                for(int i = 0;i<mailBoxVec.size();i++){

                  xMidpointDiff = fabs((xMidpoint + currX) - mailBoxVec[i].midX);
                  yMidpointDiff = fabs((yMidpoint + currY) - mailBoxVec[i].midY);

                  // ROS_INFO("MidPointDiff = %f,%f",xMidpointDiff,yMidpointDiff);
                  if(xMidpointDiff > acceptableCenterError){
                    
                    ROS_INFO_STREAM("Adding MAILBOX xmidmidpoint,ypoint" << xMidpoint + currX << "," << yMidpoint + currY);
                  
                    mb.midX = xMidpoint + currX;
                    mb.midY = yMidpoint + currY;
                    mailBoxVec.push_back(mb);
                    printFoundObjects();
                    return 0;
                  }
                }
               }
               else{
                  ROS_INFO_STREAM("Adding the FIRST MAILBOX xmidmidpoint,ypoint" << xMidpoint + currX << "," << yMidpoint + currY);
                   
                    mb.midX = xMidpoint + currX;
                    mb.midY = yMidpoint + currY;
                    mailBoxVec.push_back(mb);
                    printFoundObjects();
               }
              }
            }
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
    ptCloudFiltered.header.stamp = Time::now();

    scan_pub2_.publish(ptCloudFiltered);

    sensor_msgs::convertPointCloud2ToPointCloud(ptCloudFiltered,filteredCloud);
    
    // checks if table
    // for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it){
    //     ROS_INFO_STREAM(cluster_indices.begin()->points.x);
        
    // }

    cloud_filtered->points[0].x;
    for (int i = 0; i < filteredCloud.points.size();i++){
        //ROS_INFO("cloud_filtered->points[%d/%d].xy = %f,%f",i ,cloud_filtered->points.size(), cloud_filtered->points[i].x,cloud_filtered->points[i].y);
        tableMaybe(filteredCloud.points[i].x, filteredCloud.points[i].y,&filteredCloud);// TODO may need to change filteredCloud
        maybeMailbox(filteredCloud.points[i].x, filteredCloud.points[i].y,&filteredCloud);// TODO may need to change filteredCloud

    }
    // tableMaybe(-8.0,-2.0,&cloud_filtered);

    scan_pub4_.publish(filteredCloud);
  }
    pcl::toROSMsg(*cloud_filtered,ptCloudAux);
    scan_pub3_.publish(ptCloudAux);

    scan_pub_.publish(cloud);
    
  }
};

int main(int argc, char** argv){
  ros::init(argc, argv, "rec");
  ros::NodeHandle n;
  LaserScanToPointCloud lstopc(n);
  Subscriber mapSub = n.subscribe("map", 1000, mapConvert);
  Subscriber acmlSub = n.subscribe("/amcl_pose", 2000, &amclReceived);

 

  

  spin();

  return 0;
}
