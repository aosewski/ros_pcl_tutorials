#include <ros/ros.h>
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

ros::Publisher pub;

void cloudCallback(const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  pcl::PCLPointCloud2::Ptr            cloud_blob(new pcl::PCLPointCloud2());
  pcl::PCLPointCloud2::Ptr            cloud_filtered_blob(new pcl::PCLPointCloud2());
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_p(new pcl::PointCloud<pcl::PointXYZ>);

  pcl_conversions::toPCL(*cloud_msg, *cloud_blob);

  ROS_INFO_STREAM("PointCloud before filtering: " 
      << cloud_blob->width * cloud_blob->height << " data points.");

  // Create the filtering object: downsample the dataset using a leaf size of 1cm
  pcl::VoxelGrid<pcl::PCLPointCloud2> vg_filter;
  vg_filter.setInputCloud(cloud_blob);
  vg_filter.setLeafSize(0.01f, 0.01f, 0.01f);
  vg_filter.filter(*cloud_filtered_blob);

  ROS_INFO_STREAM("PointCloud after filtering: " 
      << cloud_filtered_blob->width * cloud_filtered_blob->height << " data points.");

  // Convert to the templated PointCloud
  pcl::fromPCLPointCloud2(*cloud_filtered_blob, *cloud_filtered);

  pcl::ModelCoefficients::Ptr         coefficients(new pcl::ModelCoefficients());
  pcl::PointIndices::Ptr              inliers(new pcl::PointIndices());
  // Create the segmentation object
  pcl::SACSegmentation<pcl::PointXYZ> seg;
  // Optional
  seg.setOptimizeCoefficients(true);
  // Mandatory
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setMaxIterations(1000);
  seg.setDistanceThreshold(0.01);

  // Create the filtering object
  pcl::ExtractIndices<pcl::PointXYZ> extract;

  // Segment the largest planar component from the cloud
  seg.setInputCloud(cloud_filtered);
  seg.segment(*inliers, *coefficients);
  if (inliers->indices.size() == 0)
  {
    ROS_ERROR_STREAM("Could not estimate a planar model for the given dataset.");
  }

  // Extract the inliers
  extract.setInputCloud(cloud_filtered);
  extract.setIndices(inliers);
  extract.setNegative(false);
  extract.filter(*cloud_p);
  ROS_INFO_STREAM("PointCloud representing the planar component: "
    << cloud_p->width * cloud_p->height << " data points.");

  // Convert to ROS data type
  sensor_msgs::PointCloud2 output;
  pcl::toROSMsg(*cloud_p, output);

  // Publish the data
  pub.publish(output);
}

int main(int argc, char** argv)
{
  // Initialize ROS
  ros::init(argc, argv, "extract_indices_node");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe("input", 1, cloudCallback);

  // Create a ROS publisher for the output point cloud 
  pub = nh.advertise<sensor_msgs::PointCloud2>("output", 1);

  // Spin
  ros::spin();
}

