/**
 * @file pc_proc_base.cpp
 * @brief This file includes the initial ROS node implementation.
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
  ros::Publisher pc_pub_;

public:
  PointCloudProcessor(ros::NodeHandle& n):
    nh_(n)
  {
    pc_sub_ = nh_.subscribe("input_cloud", 1, &PointCloudProcessor::point_cloud_callback, this);
    pc_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("output_cloud", 1);
  }

  ~PointCloudProcessor() {}

  void point_cloud_callback(const sensor_msgs::PointCloud2::ConstPtr& msg)
  {

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg (*msg, *cloud);

    // Do something with cloud and store the result in cloudOut

    //Publish point clouds
    sensor_msgs::PointCloud2 cloudOut;
    pcl::toROSMsg(*cloud, cloudOut);
    pc_pub_.publish(cloudOut);
  }

};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pc_proc_base");
  ros::NodeHandle n("~");
  PointCloudProcessor pc(n);
  ros::spin();
  return 0;
}
