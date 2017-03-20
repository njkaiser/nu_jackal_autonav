// ROUGH OUTLINE BORROWED FROM RITWICK'S pcl_495 PACKAGE:
// https://github.com/ritwik1993/pcl_495/blob/master/src/pcl_node_w_nodelets.cpp

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <boost/make_shared.hpp>
#include <tf/transform_broadcaster.h>
#include <cmath>



class FloorFinder
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

  // these need to be available to multiple methods
  pcl::ModelCoefficients::Ptr floor_coefficients;


public:
  FloorFinder()
  {
    // create publisher for output point cloud
    pub = nh.advertise<sensor_msgs::PointCloud2>("output", 1);

    // subscribe to input point cloud
    sub = nh.subscribe("/velodyne_points/voxeled", 1, &FloorFinder::cloud_cb, this);

    // callback to publish ground plane estimate
    tmr = nh.createTimer(ros::Duration(0.02), &FloorFinder::tf_cb, this); // 50 hz to match the rest of the tf tree

    floor_coefficients = pcl::ModelCoefficients::Ptr(new pcl::ModelCoefficients());
  }


private:
  void ransac(const sensor_msgs::PointCloud2ConstPtr& input, pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_projected)
  {
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointIndices::Ptr floor_indices(new pcl::PointIndices());
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
    std::cout << *floor_coefficients << std::endl;
  }

  void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
  {
    // run ransac to find floor
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_projected(new pcl::PointCloud<pcl::PointXYZI>);
    ransac(input, cloud_projected);
    pub.publish(*cloud_projected);
  }

  void tf_cb(const ros::TimerEvent& event)
  {
    // calculate ground plane triax
    if(floor_coefficients->values.size())
    {
      float nx = floor_coefficients->values[0];
      float ny = floor_coefficients->values[1];
      float nz = floor_coefficients->values[2];
      float d = floor_coefficients->values[3];

      float roll = atan2(nz, ny) + 1.57079; // 90 deg offset
      float pitch = atan2(nz, nx) + 1.57079; // 90 deg offset
      q.setRPY(roll, pitch, 1.57079); // 90 deg offset

      floor_frame_tf.setOrigin(tf::Vector3(-0.12, 0.0, -0.338));
      floor_frame_tf.setRotation(q);
      br.sendTransform(tf::StampedTransform(floor_frame_tf, ros::Time::now(), "velodyne", "floor_estimate"));
    }
    else
    {
      std::cout << "WARNING: Ground plane model coefficient array is empty (FILE = floor_finder.cpp)" << std::endl;
    }
  }

}; // END OF FloorFinder CLASS



int main(int argc, char** argv)
{
  // Initialize ROS
  ros::init(argc, argv, "floor_estimater");

  // instantiate class for storing/accessing tf data
  FloorFinder flr_est;

  ros::spin();
}
