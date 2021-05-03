

#ifndef ATTRACTOR_GUIDED_NAVIGATION_AGRIBOT_LOCAL_PLANNER_H_
#define ATTRACTOR_GUIDED_NAVIGATION_AGRIBOT_LOCAL_PLANNER_H_

#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Bool.h>
#include <tf/transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <base_local_planner/odometry_helper_ros.h>
#include <dynamic_reconfigure/server.h>
#include <nav_core/base_local_planner.h>

#include <fstream>
#include <string>
#include <vector>

#include "agribot_local_planner/AgribotLocalPlannerConfig.h"

namespace agribot_local_planner {

class AgribotLocalPlanner : public nav_core::BaseLocalPlanner {
 public:
  AgribotLocalPlanner();
  ~AgribotLocalPlanner();

  // base local planner plugin functions
  void initialize(std::string name, tf2_ros::Buffer *tf,
                  costmap_2d::Costmap2DROS *costmap_ros);

  bool setPlan(const std::vector<geometry_msgs::PoseStamped> &global_plan);

  void publishPlan(const std::vector<geometry_msgs::PoseStamped>& path, const ros::Publisher& pub);
  double getGoalPositionDistance(const geometry_msgs::PoseStamped& global_pose, double goal_x, double goal_y);
  double getGoalOrientationAngleDifference(const geometry_msgs::PoseStamped& global_pose, double goal_th);

  bool isGoalReached();

  std::vector<double> getEulerAngles(geometry_msgs::PoseStamped& Pose);

  double LinearPIDController(nav_msgs::Odometry& base_odometry, double next_t_x, double next_t_y);
  double AngularPIDController(nav_msgs::Odometry& base_odometry, double target_th_w, double robot_orien);

  bool computeVelocityCommands(geometry_msgs::Twist &cmd_vel);

 private:
  // callbacks
  void emergencyStopCallback(const std_msgs::Bool::ConstPtr &stop_msg);

  // actions
  void robotStops() {
    goal_reached_ = true;
    ROS_INFO("Robot will stop.");
  }

  // utils
  void getTransformedPosition(geometry_msgs::PoseStamped &pose, double *x,double *y, double *theta) {

    std::string &_base_frame = base_frame_;
    geometry_msgs::TransformStamped ps = tf_->lookupTransform( _base_frame, pose.header.frame_id, ros::Time(0)  );
    *x = ps.transform.translation.x;
    *y = ps.transform.translation.y;
    *theta = tf::getYaw(ps.transform.rotation);
  }

  costmap_2d::Costmap2DROS* costmap_ros_; ///< @brief The ROS wrapper for the costmap the controller will use
  costmap_2d::Costmap2D* costmap_; ///< @brief The costmap the controller will use

   tf::Vector3 robot_curr_pose;
   double robot_curr_orien;
  std::vector<double> final_orientation;

  tf2_ros::Buffer* tf_;

  base_local_planner::OdometryHelperRos *odom_helper_;
  // publishers
  ros::Publisher target_pose_pub_, curr_pose_pub;
  // subscribers
  ros::Subscriber emergency_stop_sub_;
  //ros::Subscriber odom_sub;

  // input params
  std::string base_frame_;
  bool initialized_, goal_reached_,rotating_to_goal_;
  int plan_index_, last_plan_index_;
  std::vector<geometry_msgs::PoseStamped> global_plan_;

  double p_window_, o_window_;
  double p_precision_, o_precision_;

  double max_vel_lin_, min_vel_lin_, max_incr_lin_;
  double max_vel_ang_, min_vel_ang_, max_incr_ang_;
  double k_p_lin_, k_i_lin_, k_d_lin_;
  double k_p_ang_, k_i_ang_, k_d_ang_;
  double d_t_;

  ros::Time last_time;

  double error_lin_, error_ang_;
  double integral_lin_, integral_ang_;

  // dynamic reconfigure
  dynamic_reconfigure::Server<AgribotLocalPlannerConfig> *dsrv_;
  void reconfigureCB(AgribotLocalPlannerConfig &config, uint32_t level);
};
}  // namespace agribot_local_planner

#endif  // ATTRACTOR_GUIDED_NAVIGATION_AGRIBOT_LOCAL_PLANNER_H_
