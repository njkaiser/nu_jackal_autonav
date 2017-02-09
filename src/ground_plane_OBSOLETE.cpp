#include <ros/ros.h>
#include <iostream>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/PCLPointCloud2.h>
#include <tf/transform_broadcaster.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>

#include <pcl/point_types.h>
#include <velodyne_pointcloud/point_types.h> // velodyne_pointcloud::PointXYZIR
#include <velodyne_pointcloud/rawdata.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>

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

#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/project_inliers.h>

#include <pcl/surface/mls.h>
#include <pcl/surface/concave_hull.h>

#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
// #include <boost/thread/thread.hpp>

// #include <Quaternion.h>


void ring_filter(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
{
  pcl::PassThrough<pcl::PointXYZI> pass;
  // pass.setInputCloud(cloud);
  pass.setInputCloud(cloud);
  pass.setFilterFieldName("x");
  pass.setFilterLimits(-1.5, 1.5); // keep only ring 1 (maybe ring 0? not sure yet..)

  // THIS IS NOT WORKING SINCE setFilterFieldName, WHEN SET TO "ring", DOES
  // NOT WORK. THE FILTER TEMPLATE IS NOT SET UP TO HANDLE velodyne_pointcloud::PointXYZIR, THEREFORE THIS FILTER WILL NOT WORK UNLESS
  // I MAKE MY OWN FILTER OR FIGURE OUT HOW TO FILTER BY RING SOME OTHER WAY...

  pass.filter(*cloud);
}



void box_filt(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)//, std::string filt_criteria, int max, int min)
{
  // int dims = 3;
  // std::string names[] = {"x", "y", "z"};
  // int mins[dims] = {-4.0, -4.0, -0.1};
  // int maxs[dims] = {4.0, 4.0, 0.1};
  // for(int c = 0; c < dims; ++c)
  // {
    // pcl::PassThrough<pcl::PointXYZI> pass;
    // pass.setInputCloud(cloud);
    // pass.setFilterFieldName(names[c]);
    // pass.setFilterLimits(mins[c], maxs[c]);
    // pass.filter(*cloud);

    const Eigen::Vector4f & mins = Eigen::Vector4f(-4.0, -4.0, -0.1, -99.9);
    const Eigen::Vector4f & maxs = Eigen::Vector4f(4.0, 4.0, 0.1, 99.9);

    // pcl::CropBox<pcl::PointCloud<pcl::PointXYZI> > cb;
    pcl::CropBox<pcl::PointXYZI> cb;
    cb.setMax(maxs);
    cb.setMin(mins);
    cb.setInputCloud(cloud);
    cb.filter(*cloud);
    // cb.filter(filteredIndices);
  // }
}


// void noise_filter(const pcl::PointCloud<pcl::PointXYZI>::ConstPtr &cloud, pcl::PointCloud<pcl::PointXYZI>::ConstPtr::Ptr &cloud_filtered)
// {
//   pcl::StatisticalOutlierRemoval<pcl::PointXYZI> SOR_filt;
//   SOR_filt.setInputCloud(cloud);
//   SOR_filt.setMeanK(50);
//   SOR_filt.setStddevMulThresh(1.0);
//   SOR_filt.pcl::Filter<pcl::PointXYZI>::filter(*cloud_filtered);
// }

// void input_cb(const sensor_msgs::PointCloud2ConstPtr& input)
// {
//
//   std::cout << "COUNT: " << i++ << std::endl;
//   pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
//   pcl::fromROSMsg(*input, *cloud);
//
//   // pcl::PCLPointCloud2 pcl_pc;
//   // pcl_conversions::toPCL(*input, pcl_pc);
//
//   // pcl::PointCloud<pcl::PointXYZ> cloud;
//   //
//   // pcl::fromPCLPointCloud2(pcl_pc, cloud);
//   // pcl::YOUR_PCL_FUNCTION(cloud,...);
//
//   // std::cout << input->header << std::endl;
//   // std::cout << cloud->header << std::endl;
//   // std::cout << input->fields[0] << std::endl;
//   // std::cout << input->fields[1] << std::endl;
//   // std::cout << input->fields[2] << std::endl;
//   // std::cout << input->fields[3] << std::endl;
//   // std::cout << input->fields[4] << std::endl; // this was NOT causing segfault when I checked, even though I'm currently using PointXYZI instead of PointXYZIR and there should be no field 4 entry...
//
//   // ring_filter(cloud);
//   box_filt(cloud);
//
//   std::cout << "input width: " << input->width << std::endl;
//   std::cout << "cloud width: " << cloud->width << std::endl;
//
//   // ransac(input, cloud_new);
//
//   // const sensor_msgs::PointCloud2 output;
//   sensor_msgs::PointCloud2::Ptr toROSMsg(pcl::PointCloud<pcl::PointXYZI>& output);
//   // pcl::toROSMsg(&(*cloud), &output);
//   // IS THERE ANY WAY (BESIDES GLOBAL VARIABLE) TO PASS THIS IN?
//   ros::Publisher cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/ground_plane", 1);
//   cloud_pub.publish(output);
// }


class GroundPlane
{
private:
  ros::NodeHandle nh;
  ros::Publisher cloud_pub;
  ros::Subscriber input_sub;
  ros::Subscriber tf_sub;

public:
  GroundPlane() // constructor
  {
    cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/ground_plane", 1);

    input_sub = nh.subscribe("/velodyne_points", 1, &GroundPlane::input_cb, this); // subscribe to input PointCloud2 data

    // tf_sub = nh.subscribe("velodyne", 1, &GroundPlane::tf_cb);

  } // end of constructor

  void input_cb(const sensor_msgs::PointCloud2ConstPtr& input)
  {
    // static tf::TransformBroadcaster br;
    // tf::Transform transform;
    // transform.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );
    // tf::Quaternion q;
    // q.setRPY(0, 0, 0);
    // transform.setRotation(q);
    // br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "velodyne", "pointcloud_manip"));

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*input, *cloud);

    // pcl::PCLPointCloud2 pcl_pc;
    // pcl_conversions::toPCL(*input, pcl_pc);

    // pcl::PointCloud<pcl::PointXYZ> cloud;
    //
    // pcl::fromPCLPointCloud2(pcl_pc, cloud);
    // pcl::YOUR_PCL_FUNCTION(cloud,...);

    // std::cout << input->header << std::endl;
    // std::cout << cloud->header << std::endl;
    // std::cout << input->fields[0] << std::endl;
    // std::cout << input->fields[1] << std::endl;
    // std::cout << input->fields[2] << std::endl;
    // std::cout << input->fields[3] << std::endl;
    // std::cout << input->fields[4] << std::endl; // this was NOT causing segfault when I checked, even though I'm currently using PointXYZI instead of PointXYZIR and there should be no field 4 entry...

    // ring_filter(cloud);
    box_filt(cloud);
    // std::cout << cloud->data.x << std::endl;
    // for (size_t i = 0; i < cloud->points.size (); ++i)
    // std::cout << " x: " << cloud->points[i].x
    //           << " y: " << cloud->points[i].y
    //           << " z: " << cloud->points[i].z
    //           << " I: " << cloud->points[i].intensity;
    // std::cout << std::endl;

    std::cout << "input width: " << input->width << "\tcloud width: " << cloud->width << std::endl;

    // ransac(input, cloud_new);

    // const sensor_msgs::PointCloud2 output;
    // sensor_msgs::PointCloud2 output;
    // sensor_msgs::PointCloud2::Ptr toROSMsg(pcl::PointCloud<pcl::PointXYZI>& output);
    // output.header.frame_id = "velodyne"; // same frame as velodyne data
    // output.header.stamp = ros::Time::now().toNSec();
    // output.height = output.width = 1;
    // output.points.push_back (pcl::PointXYZ(1.0, 2.0, 3.0));


    // pcl::toROSMsg(&(*cloud), &output);
    // IS THERE ANY WAY (BESIDES GLOBAL VARIABLE) TO PASS THIS IN?
    // ros::Publisher cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/ground_plane", 1);
    // cloud_pub.publish(output);
    cloud_pub.publish(cloud);
  } // end of input_cb()


  // void tf_cb(void)//const turtlesim::PoseConstPtr& msg){
  // {
  // static tf::TransformBroadcaster br;
  // tf::Transform transform;
  // transform.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );
  // tf::Quaternion q;
  // q.setRPY(0, 0, 0);
  // transform.setRotation(q);
  // br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "velodyne", "pointcloud_manip"));
  // }

}; // end of class GroundPlane



int main(int argc, char** argv)
{
  ros::init(argc, argv, "pointcloud_manip"); // initialize ROS node
  // ros::NodeHandle nh; // create node handle

  GroundPlane gp;

  // ros::Subscriber sub = nh.subscribe ("/velodyne_points", 1, input_cb); // subscribe to input PointCloud2 data

  // pub_obj = nh.advertise<sensor_msgs::PointCloud2> ("objects", 1);

  ros::spin();
  return 0;
} // end of main()
