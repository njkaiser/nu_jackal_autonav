#include "winter_project/custom_planner.h"

// copied from DWA planner:
// #include <dwa_local_planner/dwa_planner_ros.h> //this is now custom_planner.h
#include <Eigen/Core>
#include <cmath>
#include <ros/console.h>
#include <base_local_planner/goal_functions.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Twist.h>

//register this planner as a BaseLocalPlanner plugin
#include <pluginlib/class_list_macros.h> // required for loading as a plugin
PLUGINLIB_EXPORT_CLASS(custom_planner::CustomPlanner, nav_core::BaseLocalPlanner)

using namespace std;

namespace custom_planner
{

  // void DWAPlannerROS::reconfigureCB(DWAPlannerConfig &config, uint32_t level) {
  //     if (setup_ && config.restore_defaults) {
  //       config = default_config_;
  //       config.restore_defaults = false;
  //     }
  //     if ( ! setup_) {
  //       default_config_ = config;
  //       setup_ = true;
  //     }
  //
  //     // update generic local planner params
  //     base_local_planner::LocalPlannerLimits limits;
  //     limits.max_trans_vel = config.max_trans_vel;
  //     limits.min_trans_vel = config.min_trans_vel;
  //     limits.max_vel_x = config.max_vel_x;
  //     limits.min_vel_x = config.min_vel_x;
  //     limits.max_vel_y = config.max_vel_y;
  //     limits.min_vel_y = config.min_vel_y;
  //     limits.max_rot_vel = config.max_rot_vel;
  //     limits.min_rot_vel = config.min_rot_vel;
  //     limits.acc_lim_x = config.acc_lim_x;
  //     limits.acc_lim_y = config.acc_lim_y;
  //     limits.acc_lim_theta = config.acc_lim_theta;
  //     limits.acc_limit_trans = config.acc_limit_trans;
  //     limits.xy_goal_tolerance = config.xy_goal_tolerance;
  //     limits.yaw_goal_tolerance = config.yaw_goal_tolerance;
  //     limits.prune_plan = config.prune_plan;
  //     limits.trans_stopped_vel = config.trans_stopped_vel;
  //     limits.rot_stopped_vel = config.rot_stopped_vel;
  //     planner_util_.reconfigureCB(limits, config.restore_defaults);
  //
  //     // update dwa specific configuration
  //     dp_->reconfigure(config);
  // }


  // constructor - probably leave empty, do any necessary initializations in initialize()
  CustomPlanner::CustomPlanner()
  : initialized(false)//, odom_helper_("odom")
  {
    // ROS_INFO("empty constructor called");
    cout << "empty constructor called" <<endl;

    // base_local_planner::Trajectory traj;
    // cout << traj.xv_ << endl;
    // cout << traj.yv_ << endl;
    // cout << traj.thetav_ << endl;
    // cout << traj.time_delta_ << endl;
    // cout << traj.x_pts_.size() << endl;
  }


  // constructs the local planner (but technically this is not the constructor)
  void CustomPlanner::initialize(std::string name, tf::TransformListener* tf, costmap_2d::Costmap2DROS* costmap_ros)
  {
    cout << "initialize() function called" << endl;
    if(!initialized)
    {
      ros::NodeHandle private_nh("~/" + name);
      g_plan_pub_ = private_nh.advertise<nav_msgs::Path>("global_plan", 1);
      l_plan_pub_ = private_nh.advertise<nav_msgs::Path>("local_plan", 1);
      tf_ = tf;
      costmap_ros_ = costmap_ros;
      costmap_ros_->getRobotPose(current_pose_);

      // make sure to update the costmap we'll use for this cycle
      costmap_2d::Costmap2D* costmap = costmap_ros_->getCostmap();

      planner_util_.initialize(tf, costmap, costmap_ros_->getGlobalFrameID());

      //create the actual planner that we'll use.. it'll configure itself from the parameter server
      // dp_ = boost::shared_ptr<dwa_local_planner::DWAPlanner>(new dwa_local_planner::DWAPlanner(name, &planner_util_));
      // dp_ = boost::shared_ptr<CustomPlanner>(new CustomPlanner(name, &planner_util_));

      if(private_nh.getParam("odom_topic", odom_topic_))
      {
        odom_helper_.setOdomTopic(odom_topic_);
      }
      cout << "odom_topic_: " << odom_topic_ << endl;

      initialized = true;
      cout << "initialized set to true, shouldn't see this message again" << endl;

      // dsrv_ = new dynamic_reconfigure::Server<DWAPlannerConfig>(private_nh);
      // dynamic_reconfigure::Server<DWAPlannerConfig>::CallbackType cb = boost::bind(&DWAPlannerROS::reconfigureCB, this, _1, _2);
      // dsrv_->setCallback(cb);
    }
    else
    {
      ROS_WARN("This planner has already been initialized, doing nothing.");
    }
  }


  // set the [global] plan that the local planner is following
  bool CustomPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan)
  {
    if(!initialized)
    {
      ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
      return false;
    }
    //when we get a new plan, we also want to clear any latch we may have on goal tolerances
    latchedStopRotateController_.resetLatching();

    ROS_INFO("Got new plan");

    cout << "global plan size: " << orig_global_plan.size() << endl;
    // for(int i = 0; i < orig_global_plan.size(); ++i)
    // {
    //   cout << "global plan[" << i << "]: " << orig_global_plan[i] << endl;
    // }
    // return dp_->setPlan(orig_global_plan);
    // return planner_util_->setPlan(orig_global_plan);
    return planner_util_.setPlan(orig_global_plan);
    // return true; // just getting this to work for now, fill in later
    // return false; // just getting this to work for now, fill in later
  }


  // notifies nav_core if goal is reached
  bool CustomPlanner::isGoalReached()
  {
    if(!initialized)
    {
      ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
      return false;
    }

    if(!costmap_ros_->getRobotPose(current_pose_))
    {
      ROS_ERROR("Could not get robot pose");
      return false;
    }

    if(latchedStopRotateController_.isGoalReached(&planner_util_, odom_helper_, current_pose_))
    {
      ROS_INFO("Goal reached");
      return true;
    }
    else
    {
      return false;
    }
  }


  // given the current position, orientation, and velocity of the robot, compute velocity commands to send to the base
  bool CustomPlanner::computeVelocityCommands(geometry_msgs::Twist& cmd_vel)
  {
    // dispatches to either dwa sampling control or stop and rotate control, depending on whether we are close enough to goal
    cmd_vel.linear.x = 1.0;
    if(!costmap_ros_->getRobotPose(current_pose_))
    {
      ROS_ERROR("Could not get robot pose");
      return false;
    }

    std::vector<geometry_msgs::PoseStamped> transformed_plan;
    if(!planner_util_.getLocalPlan(current_pose_, transformed_plan))
    {
      ROS_ERROR("Could not get local plan");
      return false;
    }

    //if the global plan passed in is empty... we won't do anything
    if(transformed_plan.empty())
    {
      ROS_WARN_NAMED("custom_planner", "Received an empty transformed plan.");
      return false;
    }

    ROS_DEBUG_NAMED("custom_planner", "Received a transformed plan with %zu points.", transformed_plan.size());

    // update plan in dwa_planner even if we just stop and rotate, to allow checkTrajectory
    // dp_->updatePlanAndLocalCosts(current_pose_, transformed_plan);

    if(latchedStopRotateController_.isPositionReached(&planner_util_, current_pose_))
    {
      //publish an empty plan because we've reached our goal position
      std::vector<geometry_msgs::PoseStamped> local_plan;
      std::vector<geometry_msgs::PoseStamped> transformed_plan;
      publishGlobalPlan(transformed_plan);
      publishLocalPlan(local_plan);
      base_local_planner::LocalPlannerLimits limits = planner_util_.getCurrentLimits();
      // return latchedStopRotateController_.computeVelocityCommandsStopRotate(
      //   cmd_vel,
      //   limits.getAccLimits(),
      //   dp_->getSimPeriod(),
      //   &planner_util_,
      //   odom_helper_,
      //   current_pose_,
      //   boost::bind(&DWAPlanner::checkTrajectory, dp_, _1, _2, _3));
      return true; // just getting this to work for now, fill in later
    }
    else
    {
      // bool isOk = dwaComputeVelocityCommands(current_pose_, cmd_vel);
      bool isOk = true; // just getting this to work for now, fill in later
      if(isOk)
      {
        publishGlobalPlan(transformed_plan);
      }
      else
      {
        ROS_WARN_NAMED("custom_planner", "Custom planner failed to produce path.");
        std::vector<geometry_msgs::PoseStamped> empty_plan;
        publishGlobalPlan(empty_plan);
      }

      // hack to get it to work:
      // out = geometry_msgs::Twist();
      // out.linear.x = 1.0;
      // out.linear.y = 0.0;
      // out.linear.z = 0.0;
      // out.angular.x = 0.0;
      // out.angular.y = 0.0;
      // out.angular.z = 1.0;

      // Fill out the local plan
      std::vector<geometry_msgs::PoseStamped> local_plan;
      // for(int i = 0; i < path.getPointsSize(); ++i)
      // {
        tf::Stamped<tf::Pose> p =
        tf::Stamped<tf::Pose>(tf::Pose(
          tf::createQuaternionFromYaw(0.0),
          tf::Point(0.0, 0.0, 0.0)),
          ros::Time::now(),
          costmap_ros_->getGlobalFrameID());
          geometry_msgs::PoseStamped pose;
          tf::poseStampedTFToMsg(p, pose);
          local_plan.push_back(pose);
      // }
      // publish information to the visualizer
      publishLocalPlan(local_plan);

      return isOk;
    }
  }


  // destructor - delete any dynamically allocated objects
  CustomPlanner::~CustomPlanner()
  {
    //make sure to clean things up
    // delete dsrv_;
  }

  // ####################################################
  // ############# BEGIN HELPER FUNCTIONS ###############
  // ####################################################

  // publish local plan
  void CustomPlanner::publishLocalPlan(std::vector<geometry_msgs::PoseStamped>& path)
  { base_local_planner::publishPlan(path, l_plan_pub_); }

  // publish global plan
  void CustomPlanner::publishGlobalPlan(std::vector<geometry_msgs::PoseStamped>& path)
  { base_local_planner::publishPlan(path, g_plan_pub_); }


}; // END OF NAMESPACE custom_planner
