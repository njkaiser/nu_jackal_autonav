// NODE TAKEN FROM pcl_495 PACKAGE:
// https://github.com/ritwik1993/pcl_495/blob/master/src/pcl_node_w_nodelets.cpp
// todo: I have no idea what I'm doing with this

#include <ros/ros.h>

#include <sensor_msgs/PointCloud2.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/Vertices.h>

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/conditional_euclidean_clustering.h>
#include <pcl/segmentation/impl/conditional_euclidean_clustering.hpp>

#include <pcl/filters/extract_indices.h>

#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/sample_consensus/method_types.h>

#include <pcl/ModelCoefficients.h>

#include <pcl/surface/mls.h>
#include <pcl/surface/concave_hull.h>

#include <pcl/filters/crop_box.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/passthrough.h>

#include <boost/thread/thread.hpp>
#include <boost/make_shared.hpp>

#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

#include <tf/transform_broadcaster.h>

#include <cmath>


// ros::init(argc, argv, "pcl_node_w_nodelets");
// ros::NodeHandle nh;
// ros::Publisher pub;
// ros::Publisher pub_obj;
// tf::TransformBroadcaster br;
// tf::Transform transform;



class Floor_Estimater
{
private:
  // exercise the demons
  ros::NodeHandle nh;
  ros::Publisher pub;
  ros::Subscriber sub;
  ros::Timer tmr;

  // set up stuff for tf / frame broadcast
  tf::TransformBroadcaster br;
  tf::Transform floor_frame_tf;
  tf::Quaternion q;

  // these need to be available to multiple functions
  // pcl::ModelCoefficients::Ptr Floor_Estimater::floor_coefficients(new pcl::ModelCoefficients());
  // pcl::PointIndices::Ptr floor_indices;
  pcl::ModelCoefficients::Ptr floor_coefficients;


public:
  Floor_Estimater()
  {
    // std::cout << "initializing Floor_Estimater class object" << std::endl;

    // create a ROS publisher for the output point cloud
    pub = nh.advertise<sensor_msgs::PointCloud2>("output", 1);
    // pub_obj = nh.advertise<sensor_msgs::PointCloud2> ("objects", 1);

    // create a ROS subscriber for the input point cloud
    sub = nh.subscribe("/velodyne_points/voxeled", 1, &Floor_Estimater::cloud_cb, this);

    // callback to publish ground plane estimate
    tmr = nh.createTimer(ros::Duration(0.02), &Floor_Estimater::tf_cb, this); // 50 hz to match the rest of the tf tree

    // initialize with best guess for floor location
    floor_frame_tf.setOrigin(tf::Vector3(-0.12, 0.0, -0.338));
    floor_frame_tf.setRotation(tf::Quaternion(0, 0, 0, 1));

    floor_coefficients = pcl::ModelCoefficients::Ptr(new pcl::ModelCoefficients());

    // std::cout << "Floor_Estimater class object initialized" << std::endl;
  }

  void ransac(const sensor_msgs::PointCloud2ConstPtr& input, pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_projected)
  {
    // std::cout << "entered ransac() function" << std::endl;

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointIndices::Ptr floor_indices(new pcl::PointIndices());
    // pcl::ModelCoefficients::Ptr floor_coefficients(new pcl::ModelCoefficients());
    pcl::fromROSMsg(*input, *cloud);

    pcl::SACSegmentation<pcl::PointXYZI> floor_finder;

    floor_finder.setOptimizeCoefficients(true);
    // floor_finder.setModelType(pcl::SACMODEL_PARALLEL_PLANE);
    // floor_finder.setModelType(pcl::SACMODEL_NORMAL_PLANE);
    floor_finder.setModelType(pcl::SACMODEL_PLANE);
    floor_finder.setMethodType(pcl::SAC_RANSAC);
    floor_finder.setMaxIterations(300);
    floor_finder.setAxis(Eigen::Vector3f(0, 0, 1));
    floor_finder.setDistanceThreshold(0.05);
    floor_finder.setEpsAngle(0.174); // 0.174 rad ~= 10 degrees
    floor_finder.setInputCloud(boost::make_shared<pcl::PointCloud<pcl::PointXYZI> >(*cloud));
    floor_finder.segment(*floor_indices, *floor_coefficients);
    // floor_finder.segment(*floor_indices, floor_coefficients);
    std::cout << *floor_coefficients << std::endl;
    // std::cout << floor_coefficients << std::endl;


    // if (floor_indices->indices.size() > 0)
    // {
    //   // Extract the floor plane inliers
    //   pcl::PointCloud<pcl::PointXYZI>::Ptr floor_points(new pcl::PointCloud<pcl::PointXYZI>);
    //   pcl::ExtractIndices<pcl::PointXYZI> extractor;
    //   extractor.setInputCloud(boost::make_shared<pcl::PointCloud<pcl::PointXYZI> >(*cloud));
    //   extractor.setIndices(floor_indices);
    //   extractor.filter(*floor_points);
    //   extractor.setNegative(true);
    //   extractor.filter(*cloud);
    //
    //   // Project the floor inliers
    //   pcl::ProjectInliers<pcl::PointXYZI> proj;
    //   proj.setModelType(pcl::SACMODEL_PLANE);
    //   proj.setInputCloud(floor_points);
    //   proj.setModelCoefficients(floor_coefficients);
    //   proj.filter(*cloud_projected);
    //
    //   floor_points->header.frame_id = "velodyne";
    //   floor_points->header.stamp = ros::Time::now().toNSec();
    // }

    // creating a kd tree object for the search method of the extraction
    // pcl::search::KdTree<pcl::PointXYZI>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZI>);
    // tree->setInputCloud (boost::make_shared<pcl::PointCloud<pcl::PointXYZI> >(*cloud));

    // new euclidean clustering code
    // std::vector<pcl::PointIndices> cluster_indices;
    // pcl::EuclideanClusterExtraction<pcl::PointXYZI> ec;
    // ec.setClusterTolerance (0.02); // 20cm
    // ec.setMinClusterSize (100);
    // ec.setMaxClusterSize (25000);
    // ec.setSearchMethod (tree);
    // ec.setInputCloud (boost::make_shared<pcl::PointCloud<pcl::PointXYZI> >(*cloud));
    // ec.extract (cluster_indices);
    // std::cout << *floor_coefficients << std::endl;


    // int j = 0;
    //
    // for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    // {
    //   pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZI>);
    //   for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); pit++)
    //   {
    //     cloud_cluster->points.push_back (cloud_projected->points[*pit]); //*
    //   }
    //   cloud_cluster->width = cloud_cluster->points.size ();
    //   cloud_cluster->height = 1;
    //   cloud_cluster->is_dense = true;
    //   cloud_cluster->header.frame_id = "camera_link";
    //   cloud_cluster->header.stamp = ros::Time::now().toNSec();
    //   pub_obj.publish(*cloud_cluster);
    //   //std::cout << "PointCloud representing the Cluster: " << cloud_cluster->points.size () << " data points." << std::endl;
    //   //std::stringstream ss;
    //   //ss << "cloud_cluster_" << j << ".pcd";
    //   j++;
    // }

  }

  void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
  {
    // std::cout << "entered cloud callback" << std::endl;

    // run ransac to find floor
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_projected(new pcl::PointCloud<pcl::PointXYZI>);
    ransac(input, cloud_projected);
    pub.publish(*cloud_projected);
    // std::cout << "width " << cloud_projected->width << std::endl;
  }

  void tf_cb(const ros::TimerEvent& event)
  {
    // std::cout << "entered tf callback" << std::endl;

    // calculate ground plane triax
    // std::cout << floor_coefficients.values[0] << std::endl;
    std::cout << floor_coefficients->values[0] << std::endl;
    // std::cout << *floor_coefficients << std::endl;
    // float nx = floor_coefficients.values[0];
    // float ny = floor_coefficients.values[1];
    // float nz = floor_coefficients.values[2];
    // float d = floor_coefficients.values[3];

    // float roll = atan2(nz, nx);
    float roll = 0;
    // float pitch = atan2(nz, ny);
    float pitch = 0;

    q.setRPY(roll, pitch, 0);

    floor_frame_tf.setOrigin(tf::Vector3(-0.12, 0.0, -0.338));
    floor_frame_tf.setRotation(q);
    br.sendTransform(tf::StampedTransform(floor_frame_tf, ros::Time::now(), "velodyne", "floor_estimate"));
  }

}; // END OF Floor_Estimater CLASS



int main(int argc, char** argv)
{
  // Initialize ROS
  ros::init(argc, argv, "floor_estimater");

  // instantiate class for storing/accessing tf data
  Floor_Estimater flr_est;

  // sit and:
  ros::spin();
}
