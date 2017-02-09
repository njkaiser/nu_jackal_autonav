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
  tf::Quaternion q;
  tf::Transform new_tf;

  // set up stuff for corrected odom message
  nav_msgs::Odometry odom_corrected;


public:
  GroundPlaneCheat()
  {
    // callback to publish tf data at constant rate
    tmr = nh.createTimer(ros::Duration(0.02), &GroundPlaneCheat::tf_cb, this); // 50 hz to match the rest of the tf tree

    // also need to publish corrected odom message
    sub = nh.subscribe("/odom", 1, &GroundPlaneCheat::odom_cb, this);
    pub = nh.advertise<nav_msgs::Odometry>("/odometry/corrected", 1);
  }

  void odom_cb(const nav_msgs::Odometry::ConstPtr& input)
  {
    // CAN'T PUBLISH OVER TOP OF ORIGINAL ODOM MESSAGE
    // SO WILL NEED TO CREATE odom_corrected OR SOMETHING LIKE THAT
    // LOOK AT AMCL PAGE, IT'S DOING THIS EXACT SAME THING

    odom_corrected = *input;
    // msg.header.frame_id = "odom"; // matches published msgs
    odom_corrected.header.frame_id = "odom";
    // msg.child_frame_id = "base_link"; // matches published msgs
    odom_corrected.child_frame_id = "odom_corrected";
    odom_corrected.pose.pose.position.z = 0;

    pub.publish(odom_corrected);
  }


  void tf_cb(const ros::TimerEvent& event)
  {
    // NEED TO SET tf DATA AS WELL
    // std::cout << "entered tf callback" << std::endl;

    // new_tf.header.stamp = current_time;
    // new_tf.header.frame_id = "odom";
    // new_tf.child_frame_id = "odom_corrected";

    q.setRPY(0, 0, 1.57079); // 90 deg offset

    new_tf.setOrigin(tf::Vector3(-0.12, 0.0, -0.338));
    new_tf.setRotation(q);

    // new_tf.transform.translation.x = 0;
    // new_tf.transform.translation.y = 0;
    // new_tf.transform.translation.z = 0.0;
    // new_tf.transform.rotation = q;

    //send the transform
    br.sendTransform(tf::StampedTransform(new_tf, ros::Time::now(), "odom", "odom_corrected"));
  }

}; // END OF GroundPlaneCheat CLASS



int main (int argc, char** argv)
{
  ros::init(argc, argv, "ground_plane_cheat");

  // ros::shutdown();

  // sit and:
  ros::spin();
  return 0;
}
