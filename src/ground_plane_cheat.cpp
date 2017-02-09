#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>



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

  // set up stuff for corrected odom message
  nav_msgs::Odometry odom_corrected;


public:
  GroundPlaneCheat() // constructor
  {
    // create publisher for corrected odometry nav messages
    pub = nh.advertise<nav_msgs::Odometry>("/odometry/corrected", 1);

    // subscribe to virgin odom message
    sub = nh.subscribe("/odometry/filtered", 1, &GroundPlaneCheat::odom_cb, this);

    // callback to publish tf data at constant rate
    tmr = nh.createTimer(ros::Duration(0.02), &GroundPlaneCheat::tf_cb, this); // 50 hz to match the rest of the tf tree
  }


private:
  void odom_cb(const nav_msgs::Odometry::ConstPtr& input)
  {
    // std::cout << "entered odometry callback" << std::endl;

    // CAN'T PUBLISH OVER TOP OF ORIGINAL ODOM MESSAGE
    // SO WILL NEED TO CREATE odom_corrected OR SOMETHING LIKE THAT
    // LOOK AT AMCL PAGE, IT'S DOING THIS EXACT SAME THING

    odom_corrected = *input;
    // msg.header.frame_id = "odom"; // matches published msgs
    odom_corrected.header.frame_id = "odom_corrected";
    // msg.child_frame_id = "base_link"; // matches published msgs
    odom_corrected.child_frame_id = "base_link";
    odom_corrected.pose.pose.position.z = 0;

    pub.publish(odom_corrected);
  }


  void tf_cb(const ros::TimerEvent& event)
  {
    // set up the transform
    q.setRPY(0, 0, 0);
    new_tf.setOrigin(tf::Vector3(0, 0, 0));
    new_tf.setRotation(q);

    //send the transform
    // br.sendTransform(tf::StampedTransform(new_tf, ros::Time::now(), "odom_corrected", "base_link"));
    br.sendTransform(tf::StampedTransform(new_tf, ros::Time::now(), "odom", "odom_corrected"));
  }

}; // END OF GroundPlaneCheat CLASS



int main (int argc, char** argv)
{
  ros::init(argc, argv, "ground_plane_cheat");

  GroundPlaneCheat gpc_ftw;

  // sit and:
  ros::spin();
  return 0;
}
