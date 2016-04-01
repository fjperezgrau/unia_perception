/**
 * @file pc_proc_full.cpp
 * @brief This file includes the full ROS node implementation.
 * @author Francisco J. Perez Grau (fjperezgrau@gmail.com)
 */

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_broadcaster.h>

#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

class PointCloudProcessor
{
  ros::NodeHandle nh_;
  ros::Subscriber pc_sub_;
  ros::Publisher pc_filtered_pub_;
  ros::Publisher pc_downsampled_pub_;
  ros::Publisher pc_foreground_pub_;
  ros::Publisher pc_det_pub_;
  tf::TransformBroadcaster tf_;

  float max_distance_;
  float voxel_size_;
  int normal_neighbors_;
  float plane_threshold_;
  float cluster_tolerance_;
  float sphere_threshold_;
  float target_radius_;

  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled_cloud_;
  pcl::PointCloud<pcl::Normal>::Ptr normals_cloud_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr foreground_cloud_;
  pcl::PointCloud<pcl::Normal>::Ptr foreground_normals_cloud_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr det_cloud_;
  pcl::ModelCoefficients::Ptr coeff_sphere_;
  bool found_;

public:
  PointCloudProcessor(ros::NodeHandle& n):
    nh_(n),
    filtered_cloud_ (new pcl::PointCloud<pcl::PointXYZ>),
    downsampled_cloud_ (new pcl::PointCloud<pcl::PointXYZ>),
    normals_cloud_ (new pcl::PointCloud<pcl::Normal>),
    foreground_cloud_ (new pcl::PointCloud<pcl::PointXYZ>),
    foreground_normals_cloud_ (new pcl::PointCloud<pcl::Normal>),
    det_cloud_ (new pcl::PointCloud<pcl::PointXYZ>),
    coeff_sphere_ (new pcl::ModelCoefficients),
    found_ (false)
  {
    //Read input parameters
    std::string cloud_topic;
    nh_.param<std::string>("cloud_topic", cloud_topic, "/input_cloud");
    nh_.param("max_distance", max_distance_, float(2.5));
    nh_.param("voxel_size", voxel_size_, float(0.02));
    nh_.param("normal_neighbors", normal_neighbors_, int(20));
    nh_.param("plane_threshold", plane_threshold_, float(0.02));
    nh_.param("cluster_tolerance", cluster_tolerance_, float(0.02));
    nh_.param("sphere_threshold", sphere_threshold_, float(0.01));
    nh_.param("target_radius", target_radius_, float(0.085));

    ROS_INFO_STREAM("Subscribed to " << cloud_topic);
    ROS_INFO_STREAM("Max distance for PassThrough filter: " << max_distance_);
    ROS_INFO_STREAM("Voxel size for VoxelGrid filter: " << voxel_size_);
    ROS_INFO_STREAM("Number of neighbors for NormalEstimation: " << normal_neighbors_);
    ROS_INFO_STREAM("Distance threshold for planes in SACSegmentation: " << plane_threshold_);
    ROS_INFO_STREAM("Tolerance for EuclideanClusterExtraction: " << cluster_tolerance_);
    ROS_INFO_STREAM("Distance threshold for spheres in SACSegmentation: " << sphere_threshold_);
    ROS_INFO_STREAM("Target radius: " << target_radius_);

    pc_sub_ = nh_.subscribe(cloud_topic, 1, &PointCloudProcessor::point_cloud_callback, this);
    pc_filtered_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("pointcloud_processor/filtered_cloud", 1);
    pc_downsampled_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("pointcloud_processor/downsampled_cloud", 1);
    pc_foreground_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("pointcloud_processor/foreground_cloud", 1);
    pc_det_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("pointcloud_processor/det_cloud", 1);
  }

  ~PointCloudProcessor() {}


  void point_cloud_callback(const sensor_msgs::PointCloud2::ConstPtr& msg)
  {
    found_ = false;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg (*msg, *cloud);

    filter(cloud, filtered_cloud_);

    downsample(filtered_cloud_, downsampled_cloud_);

    remove_planes(downsampled_cloud_, foreground_cloud_);

    search_sphere(foreground_cloud_, det_cloud_, coeff_sphere_);

    //Publish point clouds
    sensor_msgs::PointCloud2 cloudFilteredOut;
    pcl::toROSMsg(*filtered_cloud_, cloudFilteredOut);
    pc_filtered_pub_.publish(cloudFilteredOut);

    sensor_msgs::PointCloud2 cloudDownsampledOut;
    pcl::toROSMsg(*downsampled_cloud_, cloudDownsampledOut);
    pc_downsampled_pub_.publish(cloudDownsampledOut);

    sensor_msgs::PointCloud2 cloudForegroundOut;
    pcl::toROSMsg(*foreground_cloud_, cloudForegroundOut);
    pc_foreground_pub_.publish(cloudForegroundOut);

    sensor_msgs::PointCloud2 cloudDetOut;
    pcl::toROSMsg(*det_cloud_, cloudDetOut);
    pc_det_pub_.publish(cloudDetOut);

    //Publish target location
    if(found_)
    {
      tf::Transform target_tf;
      target_tf.setOrigin(tf::Vector3(coeff_sphere_->values[0],
                          coeff_sphere_->values[1], coeff_sphere_->values[2]));
      tf::Quaternion q;
      q.setRPY(0,0,0);
      target_tf.setRotation(q);
      tf_.sendTransform(tf::StampedTransform(target_tf, ros::Time::now(), msg->header.frame_id, "target"));
      ROS_INFO_STREAM("Target detected at [" << target_tf.getOrigin().x() << ", " << 
        target_tf.getOrigin().y() << ", " << target_tf.getOrigin().z() << "]");
    }
  }

  void filter(pcl::PointCloud<pcl::PointXYZ>::Ptr src, pcl::PointCloud<pcl::PointXYZ>::Ptr& dst)
  {
    //Remove NaNs
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*src, *src, indices);

    //PassThrough filter for z > 3m
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(src);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(0.0, max_distance_);
    pass.filter(*dst);
  }

  void downsample(pcl::PointCloud<pcl::PointXYZ>::Ptr src, pcl::PointCloud<pcl::PointXYZ>::Ptr& dst)
  {
    pcl::VoxelGrid<pcl::PointXYZ> vox;
    vox.setInputCloud(src);
    vox.setLeafSize(voxel_size_, voxel_size_, voxel_size_);
    vox.filter(*dst);
  }

  void estimate_normals(pcl::PointCloud<pcl::PointXYZ>::Ptr src, pcl::PointCloud<pcl::Normal>::Ptr& dst)
  {
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());

    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    ne.setSearchMethod(tree);
    ne.setInputCloud(src);
    ne.setKSearch(normal_neighbors_);
    ne.compute(*dst);
  }

  void remove_planes(pcl::PointCloud<pcl::PointXYZ>::Ptr src, pcl::PointCloud<pcl::PointXYZ>::Ptr& dst)
  {
    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(100);
    seg.setDistanceThreshold(plane_threshold_);

    pcl::PointCloud<pcl::PointXYZ>::Ptr remaining_cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud<pcl::PointXYZ>(*src, *remaining_cloud);

    int nr_points = (int) src->points.size ();
    while (remaining_cloud->points.size() > 0.2 * nr_points)
    {
      //Segment the largest planar component from the remaining cloud
      seg.setInputCloud(remaining_cloud);
      seg.segment(*inliers, *coefficients);

      if(inliers->indices.size() == 0)
        ROS_ERROR("Could not estimate a plane model for the given point cloud.");

      //Extract ground plane points
      pcl::ExtractIndices<pcl::PointXYZ> extract;
      extract.setInputCloud(remaining_cloud);
      extract.setIndices(inliers);
      extract.setNegative(true);
      extract.filter(*remaining_cloud);
    }

    pcl::copyPointCloud(*remaining_cloud, *dst);
  }

  void search_sphere(pcl::PointCloud<pcl::PointXYZ>::Ptr src, pcl::PointCloud<pcl::PointXYZ>::Ptr& dst,
                     pcl::ModelCoefficients::Ptr& coeffs)
  {

    // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud (src);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance (cluster_tolerance_);
    ec.setMinClusterSize (50);
    ec.setMaxClusterSize (25000);
    ec.setSearchMethod (tree);
    ec.setInputCloud (src);
    ec.extract (cluster_indices);

    for (size_t i=0; i<cluster_indices.size(); i++)
    {
      pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
      pcl::copyPointCloud<pcl::PointXYZ>(*src, cluster_indices[i], *cloud_cluster);

      pcl::PointCloud<pcl::Normal>::Ptr cloud_cluster_normals (new pcl::PointCloud<pcl::Normal>);
      estimate_normals(cloud_cluster, cloud_cluster_normals);

      //Look for a sphere
      pcl::SACSegmentationFromNormals<pcl::PointXYZ,pcl::Normal> seg;
      pcl::ModelCoefficients::Ptr coefficients_sphere (new pcl::ModelCoefficients);
      pcl::PointIndices::Ptr inliers_sphere (new pcl::PointIndices);
      seg.setOptimizeCoefficients (true);
      seg.setModelType (pcl::SACMODEL_SPHERE);
      seg.setMethodType (pcl::SAC_RANSAC);
      seg.setNormalDistanceWeight (5);
      seg.setMaxIterations (100);
      seg.setDistanceThreshold (sphere_threshold_);
      seg.setRadiusLimits (target_radius_ - 0.05, target_radius_ + 0.05);
      seg.setInputCloud (cloud_cluster);
      seg.setInputNormals (cloud_cluster_normals);
      seg.segment(*inliers_sphere, *coefficients_sphere);

      if(inliers_sphere->indices.size() > 0)
      {
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud(cloud_cluster);
        extract.setIndices(inliers_sphere);
        extract.setNegative(false);
        extract.filter(*dst);
        *coeffs = *coefficients_sphere;
        found_ = true;
      }
    }
  }

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pointcloud_processor");
  ros::NodeHandle n("~");
  PointCloudProcessor pc(n);
  ros::spin();
  return 0;
}
