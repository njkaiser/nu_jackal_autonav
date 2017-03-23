#include "nu_jackal_autonav/custom_planner.hpp"

// copied from DWA planner:
// #include <dwa_local_planner/dwa_planner_ros.h> //this is now custom_planner.h
#include <Eigen/Core>
#include <cmath>
#include <ros/console.h>
#include <base_local_planner/goal_functions.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Twist.h>

// register this planner as a BaseLocalPlanner plugin
#include <pluginlib/class_list_macros.h> // required for loading as a plugin
PLUGINLIB_EXPORT_CLASS(custom_planner::CustomPlanner, nav_core::BaseLocalPlanner)

using namespace std;

namespace custom_planner
{

  // constructor - leave empty, do any necessary initializations in initialize()
  CustomPlanner::CustomPlanner()
  : already_initialized(false)//, //, odom_helper_("odom")
    // BELIEVE ONE OF THE BELOW WAS CREATING THE NO-RUN ERROR WITH move_base:
    // path_costs_(planner_util_.getCostmap()),
    // goal_costs_(planner_util_.getCostmap(), 0.0, 0.0, true),
    // goal_front_costs_(planner_util_.getCostmap(), 0.0, 0.0, true),
    // alignment_costs_(planner_util_.getCostmap())
  {
    // ROS_INFO("empty constructor called");
    // cout << "empty constructor called" <<endl;

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
    // cout << "initialize() function called" << endl;
    if(!already_initialized)
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

      if(private_nh.getParam("odom_topic", odom_topic_))
      {
        odom_helper_.setOdomTopic(odom_topic_);
      }
      cout << "odom_topic_: " << odom_topic_ << endl;

      already_initialized = true;
      cout << "initialized set to true, shouldn't see this message again" << endl;

      // dsrv_->setCallback(cb);

      // STUFF RIPPED FROM DYNAMIC RECONFIGURE, SINCE I DON'T WANT TO DEAL WITH THAT RIGHT NOW
      // private_nh.param("cheat_factor", cheat_factor_, 1.0);
      // pdist_scale_ = 0.6; // default value from DWA dynamic reconfigure
      // double resolution = planner_util_.getCostmap()->getResolution();
      // // path_costs_ = base_local_planner::MapGridCostFunction(planner_util_.getCostmap());
      // path_costs_.setScale(resolution * pdist_scale_ * 0.5);
      // gdist_scale_ = 0.8; // default value from DWA dynamic reconfigure
      // // goal_costs_ = base_local_planner::MapGridCostFunction(planner_util_.getCostmap(), 0.0, 0.0, true)
      // goal_costs_.setScale(resolution * gdist_scale_ * 0.5);
      // forward_point_distance_ = 0.325; // assuming this is heading_lookahead from dynamic reconfigure?
      // // goal_front_costs_ = base_local_planner::MapGridCostFunction(planner_util_.getCostmap(), 0.0, 0.0, true)
      // goal_front_costs_.setScale(resolution * gdist_scale_ * 0.5);
      // goal_front_costs_.setXShift(forward_point_distance_);
      // goal_front_costs_.setStopOnFailure( false );
    }
    else
    {
      ROS_WARN("This planner has already been initialized, doing nothing.");
    }

    cout << "initialize() function ended" << endl;
  } // END OF FUNCTION initialize()


  // set the [global] plan that the local planner is following
  bool CustomPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped>& orig_global_plan)
  {
    // cout << "setPlan() function called" << endl;

    if(!already_initialized)
    {
      ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
      return false;
    }
    //when we get a new plan, we also want to clear any latch we may have on goal tolerances
    latchedStopRotateController_.resetLatching();

    // ROS_INFO("Got new plan");

    // cout << "global plan size: " << orig_global_plan.size() << endl;
    // for(int i = 0; i < orig_global_plan.size(); ++i)
    // {
    //   cout << "global plan[" << i << "]: " << orig_global_plan[i] << endl;
    // }
    // return dp_->setPlan(orig_global_plan);


    cout << "last point in global plan: " << orig_global_plan.back() << endl;
    cout << "current pose: (" << current_pose_.getOrigin().getX() << ' ' << current_pose_.getOrigin().getY() << ' ' << tf::getYaw(current_pose_.getRotation()) << ")" << endl;

    // THIS IS THE LINE THAT WAS REQUIRED TO GET THINGS TO WORK, CAN'T FAKE IT OUT WITH "true" OR "false"
    return planner_util_.setPlan(orig_global_plan);

    // cout << "setPlan() function ended" << endl;
  }


  // notifies nav_core if goal is reached
  bool CustomPlanner::isGoalReached()
  {
    // cout << "isGoalReached() function called" << endl;

    if(!already_initialized)
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
      // cout << "isGoalReached() function ended" << endl;
      return false;
    }

  }


  // given the current position, orientation, and velocity of the robot, compute velocity commands to send to the base
  bool CustomPlanner::computeVelocityCommands(geometry_msgs::Twist& cmd_vel)
  {
    // cout << "computeVelocityCommands() function called" << endl;

    // dispatches to either dwa sampling control or stop and rotate control, depending on whether we are close enough to goal
    if(!already_initialized)
    {
      ROS_ERROR("This planner has not been initialized, please call initialize() before using this planner");
      return false;
    }

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

    // ROS_DEBUG_NAMED("custom_planner", "Received a transformed plan with %zu points.", transformed_plan.size());

    // update plan in dwa_planner even if we just stop and rotate, to allow checkTrajectory
    // updatePlanAndLocalCosts(current_pose_, transformed_plan);

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
      // REPLACING BELOW LINE WITH COPY/PASTE OF CODE FROM dwa_planner.cpp
      // bool isOk = dwaComputeVelocityCommands(current_pose_, cmd_vel);
      // START OF MODIFIED DWAPlannerROS::dwaComputeVelocityCommands
      bool isOk;
      // { // THESE BRACKETS AREN'T NECESSARY, BUT LEAVING THEM FOR ORGANIZATION
        // dynamic window sampling approach to get useful velocity commands
        tf::Stamped<tf::Pose> robot_vel;
        odom_helper_.getRobotVel(robot_vel);

        /* For timing uncomment
        struct timeval start, end;
        double start_t, end_t, t_diff;
        gettimeofday(&start, NULL);
        */

        //compute what trajectory to drive along
        tf::Stamped<tf::Pose> drive_cmds;
        drive_cmds.frame_id_ = costmap_ros_->getBaseFrameID();

        // call with updated footprint
        // base_local_planner::Trajectory path = dp_->findBestPath(current_pose_, robot_vel, drive_cmds, costmap_ros_->getRobotFootprint());
        //ROS_ERROR("Best: %.2f, %.2f, %.2f, %.2f", path.xv_, path.yv_, path.thetav_, path.cost_);
        // base_local_planner::Trajectory path(0.0, 0.0, 0.0, 0.02, 10);
        // path.cost_ = 9.99;

        /* For timing uncomment
        gettimeofday(&end, NULL);
        start_t = start.tv_sec + double(start.tv_usec) / 1e6;
        end_t = end.tv_sec + double(end.tv_usec) / 1e6;
        t_diff = end_t - start_t;
        ROS_INFO("Cycle time: %.9f", t_diff);
        */

        // pass along drive commands
        // cmd_vel.linear.x = drive_cmds.getOrigin().getX();
        // cmd_vel.linear.y = drive_cmds.getOrigin().getY();
        // cmd_vel.angular.z = tf::getYaw(drive_cmds.getRotation());

        // MY VERSION
        cmd_vel.linear.x = 0.0;
        cmd_vel.angular.z = 0.2;

        // if we cannot move... tell someone
      //   std::vector<geometry_msgs::PoseStamped> local_plan;
      //   if(path.cost_ < 0)
      //   {
      //     ROS_DEBUG_NAMED("dwa_local_planner", "The dwa local planner failed to find a valid plan, cost functions discarded all candidates. This can mean there is an obstacle too close to the robot.");
      //     local_plan.clear();
      //     publishLocalPlan(local_plan);
      //     isOk = false;
      //   }
      //
      //   ROS_DEBUG_NAMED("dwa_local_planner", "A valid velocity command of (%.2f, %.2f, %.2f) was found for this cycle.", cmd_vel.linear.x, cmd_vel.linear.y, cmd_vel.angular.z);
      //
      //   // Fill out the local plan
      //   for(unsigned int i = 0; i < path.getPointsSize(); ++i)
      //   {
      //     double p_x, p_y, p_th;
      //     path.getPoint(i, p_x, p_y, p_th);
      //
      //     tf::Stamped<tf::Pose> p = tf::Stamped<tf::Pose>(tf::Pose(tf::createQuaternionFromYaw(p_th), tf::Point(p_x, p_y, 0.0)), ros::Time::now(), costmap_ros_->getGlobalFrameID());
      //
      //     geometry_msgs::PoseStamped pose;
      //     tf::poseStampedTFToMsg(p, pose);
      //     local_plan.push_back(pose);
      //   }
      //
      //   //publish information to the visualizer
      //   publishLocalPlan(local_plan);
      //   isOk = true;
      // } // END OF MODIFIED DWAPlannerROS::dwaComputeVelocityCommands
      //
      //
      // // bool isOk = true; // just getting this to work for now, fill in later
      // if(isOk)
      // {
      //   publishGlobalPlan(transformed_plan);
      // }
      // else
      // {
      //   ROS_WARN_NAMED("custom_planner", "Custom planner failed to produce path.");
      //   std::vector<geometry_msgs::PoseStamped> empty_plan;
      //   publishGlobalPlan(empty_plan);
      // }
      //
      // // hack to get it to work:
      // // out = geometry_msgs::Twist();
      // // out.linear.x = 1.0;
      // // out.linear.y = 0.0;
      // // out.linear.z = 0.0;
      // // out.angular.x = 0.0;
      // // out.angular.y = 0.0;
      // // out.angular.z = 1.0;
      //
      // // Fill out the local plan
      // std::vector<geometry_msgs::PoseStamped> local_plan;
      // // for(int i = 0; i < path.getPointsSize(); ++i)
      // // {
      //   tf::Stamped<tf::Pose> p =
      //   tf::Stamped<tf::Pose>(tf::Pose(
      //     tf::createQuaternionFromYaw(0.0),
      //     tf::Point(0.0, 0.0, 0.0)),
      //     ros::Time::now(),
      //     costmap_ros_->getGlobalFrameID());
      //     geometry_msgs::PoseStamped pose;
      //     tf::poseStampedTFToMsg(p, pose);
      //     local_plan.push_back(pose);
      // // }
      // // publish information to the visualizer
      // publishLocalPlan(local_plan);

      return isOk;
    }

    // cout << "computeVelocityCommands() function ended" << endl;
  } // END OF FUNCTION computeVelocityCommands()


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


  // FIND THE BEST PATH. DUH.
  // base_local_planner::Trajectory CustomPlanner::findBestPath(tf::Stamped<tf::Pose> global_pose,tf::Stamped<tf::Pose> global_vel, tf::Stamped<tf::Pose>& drive_velocities, std::vector<geometry_msgs::Point> footprint_spec)
  // {
  //   obstacle_costs_.setFootprint(footprint_spec);
  //
  //   //make sure that our configuration doesn't change mid-run
  //   boost::mutex::scoped_lock l(configuration_mutex_);
  //
  //   Eigen::Vector3f pos(global_pose.getOrigin().getX(), global_pose.getOrigin().getY(), tf::getYaw(global_pose.getRotation()));
  //   Eigen::Vector3f vel(global_vel.getOrigin().getX(), global_vel.getOrigin().getY(), tf::getYaw(global_vel.getRotation()));
  //   geometry_msgs::PoseStamped goal_pose = global_plan_.back();
  //   Eigen::Vector3f goal(goal_pose.pose.position.x, goal_pose.pose.position.y, tf::getYaw(goal_pose.pose.orientation));
  //   base_local_planner::LocalPlannerLimits limits = planner_util_->getCurrentLimits();
  //
  //   // prepare cost functions and generators for this run
  //   generator_.initialise(pos,
  //       vel,
  //       goal,
  //       &limits,
  //       vsamples_);
  //
  //   result_traj_.cost_ = -7;
  //   // find best trajectory by sampling and scoring the samples
  //   std::vector<base_local_planner::Trajectory> all_explored;
  //   scored_sampling_planner_.findBestTrajectory(result_traj_, &all_explored);
  //
  //   if(publish_traj_pc_)
  //   {
  //       base_local_planner::MapGridCostPoint pt;
  //       traj_cloud_->points.clear();
  //       traj_cloud_->width = 0;
  //       traj_cloud_->height = 0;
  //       std_msgs::Header header;
  //       pcl_conversions::fromPCL(traj_cloud_->header, header);
  //       header.stamp = ros::Time::now();
  //       traj_cloud_->header = pcl_conversions::toPCL(header);
  //       for(std::vector<base_local_planner::Trajectory>::iterator t=all_explored.begin(); t != all_explored.end(); ++t)
  //       {
  //           if(t->cost_<0)
  //               continue;
  //           // Fill out the plan
  //           for(unsigned int i = 0; i < t->getPointsSize(); ++i) {
  //               double p_x, p_y, p_th;
  //               t->getPoint(i, p_x, p_y, p_th);
  //               pt.x=p_x;
  //               pt.y=p_y;
  //               pt.z=0;
  //               pt.path_cost=p_th;
  //               pt.total_cost=t->cost_;
  //               traj_cloud_->push_back(pt);
  //           }
  //       }
  //       traj_cloud_pub_.publish(*traj_cloud_);
  //   }
  //
  //   // verbose publishing of point clouds
  //   if (publish_cost_grid_pc_) {
  //     //we'll publish the visualization of the costs to rviz before returning our best trajectory
  //     map_viz_.publishCostCloud(planner_util_->getCostmap());
  //   }
  //
  //   // debrief stateful scoring functions
  //   oscillation_costs_.updateOscillationFlags(pos, &result_traj_, planner_util_->getCurrentLimits().min_trans_vel);
  //
  //   //if we don't have a legal trajectory, we'll just command zero
  //   if (result_traj_.cost_ < 0) {
  //     drive_velocities.setIdentity();
  //   } else {
  //     tf::Vector3 start(result_traj_.xv_, result_traj_.yv_, 0);
  //     drive_velocities.setOrigin(start);
  //     tf::Matrix3x3 matrix;
  //     matrix.setRotation(tf::createQuaternionFromYaw(result_traj_.thetav_));
  //     drive_velocities.setBasis(matrix);
  //   }
  //
  //   return result_traj_;
  // } // END OF FUNCTION findBestPath()


  // void CustomPlanner::updatePlanAndLocalCosts(tf::Stamped<tf::Pose> global_pose, const std::vector<geometry_msgs::PoseStamped>& new_plan)
  // {
  //   cout << "updatePlanAndLocalCosts() function called" << endl;
  //
  //   global_plan_.resize(new_plan.size());
  //   for (unsigned int i = 0; i < new_plan.size(); ++i) {
  //     global_plan_[i] = new_plan[i];
  //   }
  //
  //   // costs for going away from path
  //   path_costs_.setTargetPoses(global_plan_);
  //
  //   // costs for not going towards the local goal as much as possible
  //   goal_costs_.setTargetPoses(global_plan_);
  //
  //   // alignment costs
  //   geometry_msgs::PoseStamped goal_pose = global_plan_.back();
  //
  //   Eigen::Vector3f pos(global_pose.getOrigin().getX(), global_pose.getOrigin().getY(), tf::getYaw(global_pose.getRotation()));
  //   double sq_dist =
  //       (pos[0] - goal_pose.pose.position.x) * (pos[0] - goal_pose.pose.position.x) +
  //       (pos[1] - goal_pose.pose.position.y) * (pos[1] - goal_pose.pose.position.y);
  //
  //   // we want the robot nose to be drawn to its final position
  //   // (before robot turns towards goal orientation), not the end of the
  //   // path for the robot center. Choosing the final position after
  //   // turning towards goal orientation causes instability when the
  //   // robot needs to make a 180 degree turn at the end
  //   std::vector<geometry_msgs::PoseStamped> front_global_plan = global_plan_;
  //   double angle_to_goal = atan2(goal_pose.pose.position.y - pos[1], goal_pose.pose.position.x - pos[0]);
  //   front_global_plan.back().pose.position.x = front_global_plan.back().pose.position.x +
  //     forward_point_distance_ * cos(angle_to_goal);
  //   front_global_plan.back().pose.position.y = front_global_plan.back().pose.position.y + forward_point_distance_ *
  //     sin(angle_to_goal);
  //
  //   goal_front_costs_.setTargetPoses(front_global_plan);
  //
  //   // keeping the nose on the path
  //   if (sq_dist > forward_point_distance_ * forward_point_distance_ * cheat_factor_) {
  //     double resolution = planner_util_.getCostmap()->getResolution();
  //     alignment_costs_.setScale(resolution * pdist_scale_ * 0.5);
  //     // costs for robot being aligned with path (nose on path, not ju
  //     alignment_costs_.setTargetPoses(global_plan_);
  //   } else {
  //     // once we are close to goal, trying to keep the nose close to anything destabilizes behavior.
  //     alignment_costs_.setScale(0.0);
  //   }
  //
  //   cout << "updatePlanAndLocalCosts() function ended" << endl;
  // }


}; // END OF NAMESPACE custom_planner
