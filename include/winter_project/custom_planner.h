#ifndef CUSTOM_LOCAL_PLANNER_ROS
#define CUSTOM_LOCAL_PLANNER_ROS

// I added:
// #include <base_local_planner.h> THIS IS PROBABLY SAME AS nav_core/base_local_planner.h
#include <geometry_msgs/Twist.h>

/** for global path planner interface */
#include <ros/ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_local_planner.h>
#include <geometry_msgs/PoseStamped.h>
#include <angles/angles.h>
#include <base_local_planner/world_model.h>
#include <base_local_planner/costmap_model.h>

// copied from DWA planner:
#include <boost/shared_ptr.hpp>
#include <boost/thread.hpp>

#include <tf/transform_listener.h>

// #include <dynamic_reconfigure/server.h>
#include <dwa_local_planner/DWAPlannerConfig.h>

#include <angles/angles.h>

#include <nav_msgs/Odometry.h>

#include <costmap_2d/costmap_2d_ros.h>
#include <nav_core/base_local_planner.h>
#include <base_local_planner/latched_stop_rotate_controller.h>
#include <base_local_planner/odometry_helper_ros.h>

#include <dwa_local_planner/dwa_planner.h>



namespace custom_planner
{

  class CustomPlanner : public nav_core::BaseLocalPlanner
  {
    public:

    // constructor - probably leave empty, do any necessary initializations in initialize()
    CustomPlanner();


    // FUNCTION: initialize()
    //  DESCRIP: constructs the local planner (technically not the constructor though)
    //   PARAMS: name - the name to give this instance of the local planner
    //           costmap_ros - the cost map to use for assigning costs to local plans
    //  RETURNS: void
    void initialize(std::string name, tf::TransformListener* tf, costmap_2d::Costmap2DROS* costmap_ros);


    // FUNCTION: computeVelocityCommands()
    //  DESCRIP: given the current position, orientation, and velocity of the robot, compute velocity commands to send to the base
    //   PARAMS: cmd_vel will be filled with the velocity command to be passed to the robot base
    //  RETURNS: true if a valid velocity command was found, false otherwise
    bool computeVelocityCommands(geometry_msgs::Twist& cmd_vel);


    // FUNCTION: setPlan()
    //  DESCRIP: set the [global] plan that the local planner is following
    //   PARAMS: plan	- the plan to pass to the local planner
    //  RETURNS: true if the plan was updated successfully, false otherwise
    bool setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan);


    // FUNCTION: isGoalReached()
    //  DESCRIP: notifies nav_core if goal is reached
    //   PARAMS: N/A
    //  RETURNS: true if achieved, false otherwise
    bool isGoalReached();


    // destructor - delete any dynamically allocated objects
    ~CustomPlanner();


private:
  // verify we only initialize once
  bool already_initialized;

  // This helper interface provides functionality that should be common to all localplanners in the move_base context. It manages the current global plan, the current motion limits, and the current costmap (local map of sensed obstacles)
  base_local_planner::LocalPlannerUtil planner_util_;

  // used to transform point clouds (per dwa_planner_ros.h)
  tf::TransformListener* tf_;

  // costmap information from nav_core
  costmap_2d::Costmap2DROS* costmap_ros_;

  // current pose information from nav_core
  tf::Stamped<tf::Pose> current_pose_;

  // publish the local plan
  void publishLocalPlan(std::vector<geometry_msgs::PoseStamped>& path);

  // publish the global plan
  void publishGlobalPlan(std::vector<geometry_msgs::PoseStamped>& path);

  // for visualisation, publishers of global and local plan
  ros::Publisher g_plan_pub_, l_plan_pub_;

  // WHAT IS THIS?!?!?! I'll figure it out later.
  base_local_planner::LatchedStopRotateController latchedStopRotateController_;

  // this class provides odometry information for ROS based robots
  base_local_planner::OdometryHelperRos odom_helper_;

  // topic name (from nav_core)
  std::string odom_topic_;

  }; // END OF CLASS CustomPlanner

}; // END OF NAMESPACE custom_planner

#endif // END OF CUSTOM_LOCAL_PLANNER_ROS
