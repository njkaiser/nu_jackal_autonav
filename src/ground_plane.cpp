#include <ros/ros.h>
#include <iostream>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/PCLPointCloud2.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>

#include <pcl/point_types.h>
#include <velodyne_pointcloud/point_types.h> // velodyne_pointcloud::PointXYZIR

#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>

// extra includes that I hope make my code work:
#include <pcl/Vertices.h>
#include <pcl/ModelCoefficients.h>

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/conditional_euclidean_clustering.h>
#include <pcl/segmentation/impl/conditional_euclidean_clustering.hpp>
#include <pcl/segmentation/extract_clusters.h>

#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>

#include <pcl/filters/extract_indices.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/project_inliers.h>

#include <pcl/surface/mls.h>
#include <pcl/surface/concave_hull.h>

// #include <boost/thread/thread.hpp>

#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>

// no idea if this will be needed in the future, why not throw it on the heap
#include <velodyne_pointcloud/rawdata.h>

// class PCLFilter
// {
// public:
//   typedef pcl::PointCloud<pcl::PointXYZI> Cloud;
//   typedef typename Cloud::ConstPtr CloudConstPtr;
//
//   void passThroughFilter(const CloudConstPtr &cloud, Cloud::Ptr &cloud_filtered, std::string filter_target, float lower_bound, float upper_bound);
//
//   void noiseRemov alFilter(const CloudConstPtr &cloud, Cloud::Ptr &cloud_filtered);
// };










// void ring_filter(const sensor_msgs::PointCloud2ConstPtr& input, pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_passthrough)
// void PCLFilter::passThroughFilter(const CloudConstPtr &cloud, Cloud::Ptr &cloud_filtered)
// void ring_filter(pcl::PointCloud<pcl::PointXYZI> &cloud)
// void ring_filter(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
// {
//   pcl::PassThrough<pcl::PointXYZI> pass;
//   pass.setInputCloud(boost::make_shared<pcl::PointCloud<pcl::PointXYZI>>(*cloud));
//   // pass.setInputCloud(cloud);
//   pass.setFilterFieldName("ring");
//   pass.setFilterLimits(-1.5, 1.5); // keep only ring 1 (maybe ring 0? not sure yet..)
//   pass.filter(*cloud);
// }


// void noise_filter(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr &cloud, pcl::PointCloud<pcl::PointXYZI>::ConstPtr::Ptr &cloud_filtered)
// {
//   pcl::StatisticalOutlierRemoval<pcl::PointXYZI> SOR_filt;
//   SOR_filt.setInputCloud(cloud);
//   SOR_filt.setMeanK(50);
//   SOR_filt.setStddevMulThresh(1.0);
//   SOR_filt.pcl::Filter<pcl::PointXYZI>::filter(*cloud_filtered);
// }

// int i = 0;
// void input_cb(const boost::shared_ptr<const sensor_msgs::PointCloud2>& input)
void input_cb(const sensor_msgs::PointCloud2ConstPtr& input)
{
  // std::cout << "COUNT: " << i++ << std::endl;
  // pcl::PCLPointCloud2 pcl_pc2;
  // pcl_conversions::toPCL(*input,pcl_pc2);
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::fromROSMsg(*input, *cloud);
  std::cout << input->header << std::endl;
  std::cout << cloud->header << std::endl;
  // std::cout << input->fields[0] << std::endl;
  // std::cout << input->fields[1] << std::endl;
  // std::cout << input->fields[2] << std::endl;
  std::cout << input->fields[3] << std::endl;
  std::cout << input->fields[4] << std::endl; // this does NOT cause segfault, even though I'm currently using PointXYZI instead of PointXYZIR
  // std::cout << cloud->inte << std::endl;

  // ring_filter(cloud);

  // std::cout << "input width: " << input->width << std::endl;
  // std::cout << "cloud width: " << cloud->width << std::endl;

  // pcl::PointCloud<velodyne_pointcloud::PointXYZIR>::Ptr cloud_new(new pcl::PointCloud<velodyne_pointcloud::PointXYZIR>);
  // std::cout << cloud_new->height << std::endl;
  // ransac(input, cloud_new);
  // pub.publish(*cloud_new);
  // velodyne_pointcloud::PointXYZI temp;
  // pcl::PointXYZI temp;
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "test"); // initialize ROS node
  ros::NodeHandle nh; // create node handle
  ros::Subscriber sub = nh.subscribe ("velodyne_points", 1, input_cb); // subscribe to input PointCloud2 data

  // Create a ROS publisher for the output point cloud
  // pub = nh.advertise<sensor_msgs::PointCloud2> ("output", 1);
  // pub_obj = nh.advertise<sensor_msgs::PointCloud2> ("objects", 1);

  ros::spin ();
  return 0;
}
