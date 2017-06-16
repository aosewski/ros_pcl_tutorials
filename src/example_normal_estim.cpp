#include <cstdlib>
#include <string>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

// PCL specific includes
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/filters/voxel_grid.h>
#ifdef _OPENMP
#include <pcl/features/normal_3d_omp.h>
#else
#include <pcl/features/normal_3d.h>
#endif
#include <pcl/visualization/cloud_viewer.h>

class CloudNormalEstimatorNode
{
public:
  typedef pcl::PointXYZ             Point;
  typedef pcl::PointCloud<Point>    PointCloud;
  typedef pcl::Normal               Normal;
  typedef pcl::PointCloud<Normal>   NormalCloud;

  CloudNormalEstimatorNode()
    :
      m_nh_priv_("~"),
      m_viewer_(pcl::visualization::PCLVisualizer("PCL Viewer")),
      m_cloud_(new PointCloud()),
      m_cloud_filtered_(new PointCloud()),
      m_cloud_normals_(new NormalCloud()),
      m_cloud_normals_id_("normals cloud"),
      m_level_(1),
      m_scale_(0.02f),
      m_vg_leaf_size_(0.0f)
  {
    m_sub_ = m_nh_.subscribe("input", 1,
      &CloudNormalEstimatorNode::cloudCallback, this);

    m_viewer_.setBackgroundColor(0.0, 0.0, 0.5);
    m_viewer_.setPointCloudRenderingProperties(
      pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, m_cloud_normals_id_);
    m_viewer_.addCoordinateSystem(1.0);
    m_viewer_.initCameraParameters();

    std::string leaf_size_key("leaf_size");
    // We have to use prive node handle to access privte node parameter
    if (!m_nh_priv_.hasParam(leaf_size_key))
    {
      ROS_ERROR("\"example_normal_estim_node\" doesn't have \"leaf_size\" \
        parameter!");
      exit(EXIT_FAILURE);
    }
    else
    {
      ROS_INFO("Node has \"leaf_size\" parameter!");
    }

    if (!m_nh_priv_.getParam(leaf_size_key, m_vg_leaf_size_))
    {
      ROS_ERROR("Couldn't get \"leaf_size\" parameter!");
      exit(EXIT_FAILURE);
    }
    else
    {
      ROS_INFO("Successfully retrived \"leaf_size\" parameter value!");
    }

    if (m_vg_leaf_size_ > 0)
    {
      m_vg_filter_.setLeafSize(m_vg_leaf_size_, m_vg_leaf_size_,
        m_vg_leaf_size_);
    }
    ROS_INFO_STREAM("leaf_size:\t" << m_vg_leaf_size_);
  };

  ~CloudNormalEstimatorNode() {};
  
private:
  // ROS stuff
  ros::NodeHandle m_nh_priv_, m_nh_;
  ros::Subscriber m_sub_;

  // PCL stuff
  pcl::visualization::PCLVisualizer         m_viewer_;
  PointCloud::Ptr                           m_cloud_;
  PointCloud::Ptr                           m_cloud_filtered_;
  #ifdef _OPENMP
  pcl::NormalEstimationOMP<Point, Normal>   m_ne_;
  #else
  pcl::NormalEstimation<Point, Normal>      m_ne_;
  #endif
  NormalCloud::Ptr                          m_cloud_normals_;
  std::string                               m_cloud_normals_id_;
  pcl::VoxelGrid<Point>                     m_vg_filter_;

  // specify density of displayed point cloud display every n'th point
  int   m_level_;
  float m_scale_;
  float m_vg_leaf_size_;

  void cloudCallback(
    const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
  {
    pcl::fromROSMsg(*cloud_msg, *m_cloud_);

    pcl::search::KdTree<Point>::Ptr tree(new pcl::search::KdTree<Point>());
    // XXX: don't need to call:
    // m_cloud_normals_->points.clear();
    // as NormalEstimator::compute method will overwrite its content

    // Perform the actual filtering
    if (m_vg_leaf_size_ > 0)
    {
      m_vg_filter_.setInputCloud(PointCloud::ConstPtr(m_cloud_));
      m_vg_filter_.filter(*m_cloud_filtered_);
    }
    else
    {
      m_cloud_filtered_ = m_cloud_;
    }

    // Create the normal estimation class, and pass the input dataset to it
    m_ne_.setInputCloud(m_cloud_filtered_);
    // pass an empty kdtree representation to the normal 
    // estimation object. Its content will be filled inside the object, 
    // based on the given input dataset (as no other search surface is given).
    m_ne_.setSearchMethod(tree);
    // Use all neighbors in a sphere of radius 3cm
    m_ne_.setRadiusSearch(0.03);
    // Compute the features
    m_ne_.compute(*m_cloud_normals_);
    // cloud_normals->points.size () should have the same size as the input m_cloud_->points.size ()*

    m_viewer_.removePointCloud(m_cloud_normals_id_);
    m_viewer_.addPointCloudNormals<Point ,Normal>(m_cloud_filtered_,
        m_cloud_normals_, m_level_, m_scale_, m_cloud_normals_id_);
    // allow viewer to run visualization loop for 50ms, and force redraw
    m_viewer_.spinOnce(100, true);
  }
};

int main(int argc, char** argv)
{
  // Initialize ROS
  ros::init(argc, argv, "example_normal_estim_node");
  
  CloudNormalEstimatorNode cne_node;
  // Spin
  ros::spin();
}
