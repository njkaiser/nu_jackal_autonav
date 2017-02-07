#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>


int main (int argc, char** argv)
{
  ros::init(argc, argv, "ground_plane_cheat");
  ros::NodeHandle nh;
  ros::Publisher pub = nh.advertise<nav_msgs::Odometry>("/odometry/filtered", 1);
  ros::Rate rate(10);
  nav_msgs::Odometry msg;

  while(ros::ok())
  {
    ros::spinOnce(); // check for incoming messages
    msg.header.seq = 1; // uint32, consecutively increasing ID
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "odom"; // matches published msgs
    msg.child_frame_id = "base_link"; // matches published msgs

    msg.pose.pose.position.z = 0;
    // BELOW IS WRONG, HEADING WOUDLN'T CHANGE IF WE SET q = < 0, 0, 0, 1 >
    // msg.pose.pose.orientation.x = 0;
    // msg.pose.pose.orientation.y = 0;
    // msg.pose.pose.orientation.z = 0;
    // msg.pose.pose.orientation.w = 1;


// NEED TO SET tf DATA AS WELL
// CAN'T PUBLISH OVER TOP OF ORIGINAL ODOM MESSAGE
// SO WILL NEED TO CREATE odom_corrected OR SOMETHING LIKE THAT
// LOOK AT AMCL PAGE, IT'S DOING THIS EXACT SAME THING


    pub.publish(msg);
    rate.sleep();
  }
  ros::shutdown();
  return 0;
}
