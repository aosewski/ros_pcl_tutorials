#include <ros/ros.h>
#include <ros/console.h>
#include <sensor_msgs/PointCloud2.h>

// PCL specific includes
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/features/normal_3d.h>

ros::Publisher plane_pub, cylinder_pub; 
typedef pcl::PointXYZ PointT;

void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  // All the objects needed
  pcl::PCDReader                                        reader;
  pcl::PassThrough<PointT>                              pass;
  pcl::NormalEstimation<PointT, pcl::Normal>            ne;
  pcl::SACSegmentationFromNormals<PointT, pcl::Normal>  seg; 
  pcl::PCDWriter                                        writer;
  pcl::ExtractIndices<PointT>                           extract;
  pcl::ExtractIndices<pcl::Normal>                      extract_normals;
  pcl::search::KdTree<PointT>::Ptr                      tree(new pcl::search::KdTree<PointT>());

  // Datasets
  pcl::PointCloud<PointT>::Ptr      cloud(new pcl::PointCloud<PointT>);
  pcl::PointCloud<PointT>::Ptr      cloud_filtered(new pcl::PointCloud<PointT>);
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
  pcl::PointCloud<PointT>::Ptr      cloud_filtered2(new pcl::PointCloud<PointT>);
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals2(new pcl::PointCloud<pcl::Normal>);
  pcl::ModelCoefficients::Ptr       coefficients_plane(new pcl::ModelCoefficients);
  pcl::ModelCoefficients::Ptr       coefficients_cylinder(new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr            inliers_plane(new pcl::PointIndices);
  pcl::PointIndices::Ptr            inliers_cylinder(new pcl::PointIndices);

  sensor_msgs::PointCloud2 ros_cloud_plane, ros_cloud_cylinder;

  // convert from ROS msg
  pcl::fromROSMsg(*cloud_msg, *cloud);

  // Build a passthrough filter to remove spurious NaNs
  // filter data points further away than 1.5m
  pass.setInputCloud (cloud);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (0, 1.5);
  pass.filter (*cloud_filtered);
  ROS_INFO_STREAM("PointCloud after filtering has: " << cloud_filtered->points.size () << " data points.");

  // Estimate point normals
  ne.setSearchMethod (tree);
  ne.setInputCloud (cloud_filtered);
  ne.setKSearch (50);
  ne.compute (*cloud_normals);

  // Create the segmentation object for the planar model and set all the parameters
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_NORMAL_PLANE);
  seg.setNormalDistanceWeight (0.1);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (100);
  seg.setDistanceThreshold (0.03);
  seg.setInputCloud (cloud_filtered);
  seg.setInputNormals (cloud_normals);
  // Obtain the plane inliers and coefficients
  seg.segment (*inliers_plane, *coefficients_plane);
  ROS_INFO_STREAM("Plane coefficients: " << *coefficients_plane);

  // Extract the planar inliers from the input cloud
  extract.setInputCloud (cloud_filtered);
  extract.setIndices (inliers_plane);
  extract.setNegative (false);

  // publish planar inliers
  //pcl::PointCloud<PointT>::Ptr cloud_plane (new pcl::PointCloud<PointT> ());
  //extract.filter (*cloud_plane);
  //ROS_INFO_STREAM("PointCloud representing the planar component: " << cloud_plane->points.size () << " data points.");
  //pcl::toROSMsg(*cloud_plane, ros_cloud_cylinder);
  //cylinder_pub.publish(ros_cloud_cylinder);
  
  // Remove the planar inliers, extract the rest
  extract.setNegative (true);
  extract.filter (*cloud_filtered2);
  extract_normals.setNegative (true);
  extract_normals.setInputCloud (cloud_normals);
  extract_normals.setIndices (inliers_plane);
  extract_normals.filter (*cloud_normals2);

  // Create the segmentation object for cylinder segmentation and set all the parameters
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_CYLINDER);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setNormalDistanceWeight (0.1);
  seg.setMaxIterations (1000);
  seg.setDistanceThreshold (0.05);
  seg.setRadiusLimits (0, 0.1);
  seg.setInputCloud (cloud_filtered2);
  seg.setInputNormals (cloud_normals2);

  // Obtain the cylinder inliers and coefficients
  seg.segment (*inliers_cylinder, *coefficients_cylinder);
  ROS_INFO_STREAM("Cylinder coefficients: " << *coefficients_cylinder);

  // Write the cylinder inliers to disk
  extract.setInputCloud (cloud_filtered2);
  extract.setIndices (inliers_cylinder);
  extract.setNegative (false);

  pcl::PointCloud<PointT>::Ptr cloud_cylinder (new pcl::PointCloud<PointT> ());
  extract.filter (*cloud_cylinder);
  if (cloud_cylinder->points.empty ()) 
    ROS_INFO_STREAM("Can't find the cylindrical component.");
  else
  {
    ROS_INFO_STREAM("PointCloud representing the cylindrical component: " << cloud_cylinder->points.size () << " data points.");
    // Convert to ROS data type
    pcl::toROSMsg(*cloud_cylinder, ros_cloud_cylinder);
    cylinder_pub.publish(ros_cloud_cylinder);
  }
}

int main(int argc, char** argv)
{
  // Initialize ROS
  ros::init(argc, argv, "extract_cylinder_node");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe("input", 1, cloudCallback);

  // Create a ROS publisher for the output point cloud 
  plane_pub = nh.advertise<sensor_msgs::PointCloud2>("plane_cloud", 10);
  cylinder_pub = nh.advertise<sensor_msgs::PointCloud2>("cylinder_cloud", 10);

  // Spin
  ros::spin();
}

