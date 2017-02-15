#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <iostream>



class GroundPlaneCheat
{
private:
  // exercise the demons
  ros::NodeHandle nh;
  ros::Publisher pub;
  ros::Subscriber sub;
  ros::Timer tmr;

  // set up stuff for tf broadcast
  tf::TransformBroadcaster br;
  tf::Transform new_tf;
  tf::Quaternion q;

  // and now for tf listen
  tf::TransformListener listener;
  tf::StampedTransform tf_in;

  // set up stuff for corrected odom message
  nav_msgs::Odometry odom_corrected;
  float z_offset;


public:
  GroundPlaneCheat() // constructor
  {
    // I don't like empty terminal screens
    std::cout << "setting up ground plane correction node...\n" << std::flush;

    // create publisher for corrected odometry nav messages
    pub = nh.advertise<nav_msgs::Odometry>("/odometry/corrected", 1);

    // subscribe to virgin odom message
    sub = nh.subscribe("/odometry/filtered", 1, &GroundPlaneCheat::topic_cb, this);

    // need odom to base_link tf info so we can cancel out the z
    std::cout << "waiting for transform from /odom to /base_link...\n" << std::flush;
    listener.waitForTransform("/odom", "/base_link", ros::Time::now(), ros::Duration(3.0));

    // callback to publish tf data at constant rate
    tmr = nh.createTimer(ros::Duration(0.02), &GroundPlaneCheat::tf_cb, this); // 50 hz to match the rest of the tf tree

    std::cout << "... done!\n" << std::flush;
  } // END OF CONSTRUCTOR


private:
  void topic_cb(const nav_msgs::Odometry::ConstPtr& input)
  {
    // std::cout << "entered odometry callback" << std::endl;

    odom_corrected = *input;
    // msg.header.frame_id = "odom"; // matches published msgs
    odom_corrected.header.frame_id = "odom_corrected";
    // msg.child_frame_id = "base_link"; // matches published msgs
    odom_corrected.child_frame_id = "base_link";
    odom_corrected.pose.pose.position.z = 0;

    pub.publish(odom_corrected);
  } // END OF topic_cb() FUNCTION


  void tf_cb(const ros::TimerEvent& event)
  {
    // grab z offset so we can correct it
    try
    { listener.lookupTransform("/odom", "/base_link", ros::Time(0), tf_in); }
    catch (tf::TransformException ex)
    {
      // ROS_ERROR("%s", ex.what());
      ROS_WARN("%s", ex.what());
      ros::Duration(1.0).sleep();
    }

    // z_offset = tf_in.transform.translation.z;
    z_offset = tf_in.getOrigin().z();

    // set up the transform variables
    q.setRPY(0, 0, 0);
    new_tf.setOrigin(tf::Vector3(0, 0, z_offset));
    new_tf.setRotation(q);

    //send the transform
    br.sendTransform(tf::StampedTransform(new_tf, ros::Time::now(), "odom", "odom_corrected"));
  } // END OF tf_cb() FUNCTION

}; // END OF GroundPlaneCheat CLASS



int main (int argc, char** argv)
{
  ros::init(argc, argv, "ground_plane_cheat");

  GroundPlaneCheat gpc_ftw;

  ros::spin();
  return 0;
} // END OF main() FUNCTION
