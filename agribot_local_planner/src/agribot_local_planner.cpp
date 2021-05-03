#include "agribot_local_planner.h"
#include <costmap_2d/costmap_2d_ros.h>
#include <pluginlib/class_list_macros.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <base_local_planner/goal_functions.h>

using namespace std;

// register planner
PLUGINLIB_EXPORT_CLASS(agribot_local_planner::AgribotLocalPlanner,nav_core::BaseLocalPlanner)

namespace agribot_local_planner {

constexpr double kControllerFrequency = 20.0;

AgribotLocalPlanner::AgribotLocalPlanner()
    : initialized_(false), goal_reached_(false) {

}

AgribotLocalPlanner::~AgribotLocalPlanner() {
  if (dsrv_) {
    delete dsrv_;
  }
}

void AgribotLocalPlanner::reconfigureCB(AgribotLocalPlannerConfig& config,uint32_t level) {
  // target
  p_window_ = config.pos_window;
  o_window_ = config.orient_window;
  // goal tolerance
  p_precision_ = config.pos_precision;
  o_precision_ = config.orient_precision;
  // linear
  max_vel_lin_ = config.max_vel_lin;
  min_vel_lin_ = config.min_vel_lin;
  max_incr_lin_ = config.max_incr_lin;
  // angular
  max_vel_ang_ = config.max_vel_ang;
  min_vel_ang_ = config.min_vel_ang;
  max_incr_ang_ = config.max_incr_ang;
  // pid controller params
  k_p_lin_ = config.k_p_lin;
  k_i_lin_ = config.k_i_lin;
  k_d_lin_ = config.k_d_lin;

  k_p_ang_ = config.k_p_ang;
  k_i_ang_ = config.k_i_ang;
  k_d_ang_ = config.k_d_ang;
}

void AgribotLocalPlanner::initialize(std::string name,tf2_ros::Buffer* tf,costmap_2d::Costmap2DROS* costmap_ros) {
  if (!initialized_) {
    ros::NodeHandle nh = ros::NodeHandle("~/" + name);
    tf_ = tf;
    odom_helper_ = new base_local_planner::OdometryHelperRos(
        "/sensor_pose_odo");  //  /sensor_pose_odo
    target_pose_pub_ = nh.advertise<geometry_msgs::PoseStamped>(
        "/target_pose", 10);  // target_pose
    curr_pose_pub = nh.advertise<geometry_msgs::PoseStamped>(
        "/current_pose", 10);  // target_pose
    // dynamic reconfigure
    dsrv_ = new dynamic_reconfigure::Server<AgribotLocalPlannerConfig>(nh);
    dynamic_reconfigure::Server<AgribotLocalPlannerConfig>::CallbackType cb =
        boost::bind(&AgribotLocalPlanner::reconfigureCB, this, _1, _2);
    dsrv_->setCallback(cb);

    base_frame_ = "/base_link";
    plan_index_ = 0;
    last_plan_index_ = 0;
    goal_reached_ = false;
    initialized_ = true;

    robot_curr_pose[0] =0;robot_curr_pose[1]=0;robot_curr_pose[2]=0;
    robot_curr_orien =0;

    costmap_ros_ = costmap_ros;
    // initialize the copy of the costmap the controller will use
    costmap_ = costmap_ros_->getCostmap();

    // time interval
    double controller_freqency;
    nh.param("/move_base/controller_frequency", controller_freqency,
             kControllerFrequency);
    d_t_ = 1 / controller_freqency;

    ROS_INFO("Agribot local planner initialized!");
  } else {
    ROS_WARN("Agribot local planner has already been initialized.");
  }
}

// plugin functions
bool AgribotLocalPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped>& global_plan){
  if (!initialized_) {
    ROS_ERROR("Agribot local planner has not been initialized.");
    return false;
  }
  // set new plan
  global_plan_.clear();
  global_plan_ = global_plan;
  // reset plan parameters
  plan_index_ = 0;
  goal_reached_ = false;
  // reset pid
  integral_lin_ = integral_ang_ = 0.0;
  error_lin_ = error_ang_ = 0.0;
  return true;
}

void AgribotLocalPlanner::publishPlan(const std::vector<geometry_msgs::PoseStamped>& path,const ros::Publisher& pub) {
  // given an empty path we won't do anything
  if (path.empty()) return;

  // create a path message
  nav_msgs::Path gui_path;
  gui_path.poses.resize(path.size());
  gui_path.header.frame_id = path[0].header.frame_id;
  gui_path.header.stamp = path[0].header.stamp;

  // Extract the plan in world co-ordinates, we assume the path is all in the
  // same frame
  for (unsigned int i = 0; i < path.size(); i++) {
    gui_path.poses[i] = path[i];
  }

  pub.publish(gui_path);
}

double AgribotLocalPlanner::getGoalPositionDistance(const geometry_msgs::PoseStamped& global_pose, double goal_x,double goal_y) {
  return hypot(goal_x - global_pose.pose.position.x,
               goal_y - global_pose.pose.position.y);
}

double AgribotLocalPlanner::getGoalOrientationAngleDifference(const geometry_msgs::PoseStamped& global_pose, double t_th_w) {
  tf::Quaternion q(
      global_pose.pose.orientation.x, global_pose.pose.orientation.y,
      global_pose.pose.orientation.z, global_pose.pose.orientation.w);
  tf::Matrix3x3 m(q);
  double roll, pitch, yaw;
  m.getRPY(roll, pitch, yaw);
  double angle = t_th_w - yaw;
  // cout << angle  << "  "  << t_th_w << "  "  << yaw << endl;
  double a = fmod(fmod(angle, 2.0 * M_PI) + 2.0 * M_PI, 2.0 * M_PI);
  if (a > M_PI) {
    a -= 2.0 * M_PI;
  }
  return a;
}

bool AgribotLocalPlanner::isGoalReached() {
  if (!initialized_) {
    ROS_ERROR("Agribot local planner has not been initialized.");
    return false;
  }
  if (goal_reached_) {
    ROS_ERROR("AgribotLocalPlanner goal reached...");
    return true;
  }
  if (plan_index_ >= global_plan_.size() - 1) {
    // last pose
    //cout << "Orinet: "<< fabs(final_orientation[2] - robot_curr_orien) << " - " << o_precision_ << "  Pose: " << getGoalPositionDistance(global_plan_.back(), robot_curr_pose[0],robot_curr_pose[1]) << " - " << p_precision_ <<  endl;
    if (fabs(final_orientation[2] - robot_curr_orien) <= o_precision_ && 
      getGoalPositionDistance(global_plan_.back(), robot_curr_pose[0],robot_curr_pose[1]) <= p_precision_) {
      goal_reached_ = true;
      robotStops();
      ROS_INFO("Goal has been reached!");
    }
  }
  return goal_reached_;
}

std::vector<double> AgribotLocalPlanner::getEulerAngles(geometry_msgs::PoseStamped& Pose) {
  std::vector<double> EulerAngles;
  EulerAngles.resize(3, 0);
  tf::Quaternion q(Pose.pose.orientation.x, Pose.pose.orientation.y,
                   Pose.pose.orientation.z, Pose.pose.orientation.w);
  tf::Matrix3x3 m(q);
  m.getRPY(EulerAngles[0], EulerAngles[1], EulerAngles[2]);
  return EulerAngles;
}

double AgribotLocalPlanner::LinearPIDController(nav_msgs::Odometry& base_odometry, double next_t_x, double next_t_y) {
  double vel_curr = hypot(base_odometry.twist.twist.linear.y,
                          base_odometry.twist.twist.linear.x);
  //cout << vel_curr << endl;
  double vel_target =
      hypot(next_t_x, next_t_y) / d_t_;  // linear velocity of depends on the p_windows (direct relation)

  if (fabs(vel_target) > max_vel_lin_) {
    vel_target = copysign(max_vel_lin_, vel_target);
  }

  double err_vel = vel_target - vel_curr;

  integral_lin_ += err_vel * d_t_;
  double derivative_lin = (err_vel - error_lin_) / d_t_;
  double incr_lin =
      k_p_lin_ * err_vel + k_i_lin_ * integral_lin_ + k_d_lin_ * derivative_lin;
  error_lin_ = err_vel;

  if (fabs(incr_lin) > max_incr_lin_) incr_lin = copysign(max_incr_lin_, incr_lin);

  double x_velocity = vel_curr + incr_lin;
  if (fabs(x_velocity) > max_vel_lin_) x_velocity = copysign(max_vel_lin_, x_velocity);
  if (fabs(x_velocity) < min_vel_lin_) x_velocity = copysign(min_vel_lin_, x_velocity);

  return x_velocity;
}

double AgribotLocalPlanner::AngularPIDController(nav_msgs::Odometry& base_odometry, double target_th_w, double robot_orien) {
  double orien_err = target_th_w - robot_orien;
  if(orien_err > M_PI) orien_err -= (2* M_PI);
  if(orien_err < -M_PI) orien_err += (2* M_PI);
  double target_vel_ang = (orien_err) / d_t_;
  if (fabs(target_vel_ang) > max_vel_ang_) {
    target_vel_ang = copysign(max_vel_ang_, target_vel_ang);
  }

  double vel_ang = base_odometry.twist.twist.angular.z;
  double error_ang = target_vel_ang - vel_ang;
  integral_ang_ += error_ang * d_t_;
  double derivative_ang = (error_ang - error_ang_) / d_t_;
  double incr_ang = k_p_ang_ * error_ang + k_i_ang_ * integral_ang_ +
                    k_d_ang_ * derivative_ang;
  error_ang_ = error_ang;

  if (fabs(incr_ang) > max_incr_ang_) incr_ang = copysign(max_incr_ang_, incr_ang);

  double th_velocity = copysign(vel_ang + incr_ang, target_vel_ang);
  if (fabs(th_velocity) > max_vel_ang_) th_velocity = copysign(max_vel_ang_, th_velocity);
  if (fabs(th_velocity) < min_vel_ang_) th_velocity = copysign(min_vel_ang_, th_velocity);

  
  return th_velocity;
}

bool AgribotLocalPlanner::computeVelocityCommands(geometry_msgs::Twist& cmd_vel) {
  if (!initialized_) {
    ROS_ERROR("PID planner has not been initialized.");
    return false;
  }
  if (goal_reached_) {
    ROS_ERROR("AgribotLocalPlanner goal reached without motion.");
    return true;
  }
  // next target
  geometry_msgs::PoseStamped target;
  geometry_msgs::PoseStamped curr_pose;

  double t_x, t_y, t_th;
  double x_vel = 0, th_vel = 0;
  double t_th_w = 0.0;

  // looking for the next point in the path far enough with minimum differene in
  // angle
  while (plan_index_ < global_plan_.size()) {
    target = global_plan_[plan_index_];
    // finding next point in path to get the orientation
    int next_plan_index = min(((int)global_plan_.size()) - 1, plan_index_ + 1);
    // getting target orientation in world coord based on next target and the
    // one after it
    t_th_w = atan2((global_plan_[next_plan_index].pose.position.y -
                    global_plan_[plan_index_].pose.position.y),
                   (global_plan_[next_plan_index].pose.position.x -
                    global_plan_[plan_index_].pose.position.x));
    // edit target adding orientation from t_th_w
    tf::Quaternion th_target_quat = tf::createQuaternionFromYaw(t_th_w);
    target.pose.orientation.x = th_target_quat[0];
    target.pose.orientation.y = th_target_quat[1];
    target.pose.orientation.z = th_target_quat[2];
    target.pose.orientation.w = th_target_quat[3];
    getTransformedPosition(target, &t_x, &t_y, &t_th);

    if (hypot(t_x, t_y) > p_window_ || fabs(t_th) > o_window_) {
      break;
    }
    plan_index_++;
    //cout << global_plan_.size() << " : " << next_plan_index << " -> " << plan_index_ << endl;
  }

  if (plan_index_ >= global_plan_.size()-1) {
    getTransformedPosition(global_plan_.back(), &t_x, &t_y, &t_th);
  }

  // invoking robot pose and orientaiton
  // tf::Stamped<tf::Pose> global_pose; // use tf2
  geometry_msgs::PoseStamped _g;
  geometry_msgs::PoseStamped& global_pose = _g;
  global_pose.header.stamp = ros::Time(0);
  costmap_ros_->getRobotPose(global_pose);
  robot_curr_pose[0] = global_pose.pose.position.x; //global_pose.getOrigin();
  robot_curr_pose[1] = global_pose.pose.position.y; //global_pose.getOrigin();
  robot_curr_pose[2] = global_pose.pose.position.z; //global_pose.getOrigin();
  robot_curr_orien = tf::getYaw(global_pose.pose.orientation);
  //********************************************************************************
  // odometry observation - getting robot velocities in robot frame
  nav_msgs::Odometry base_odom;
  odom_helper_->getOdom(base_odom);

  // get final goal orientation - Quaternion to Euler
  final_orientation = getEulerAngles(global_plan_.back());
  //cout << "Orinet: "<< final_orientation[2] - robot_curr_orien  << " Pose: " << getGoalPositionDistance(global_plan_.back(), robot_curr_pose[0],robot_curr_pose[1]) <<  endl;
  //********************************************************************************
  if (getGoalPositionDistance(global_plan_.back(), robot_curr_pose[0],robot_curr_pose[1]) <= p_precision_) {
    // check to see if the goal orientation has been reached
    if (fabs(final_orientation[2] - robot_curr_orien) <= o_precision_) {
      // set the velocity command to zero
      cmd_vel.linear.x = 0.0;
      cmd_vel.linear.y = 0.0;
      cmd_vel.angular.z = 0.0;
      rotating_to_goal_ = false;
      goal_reached_ = true; 
      cout << "pose and orientation are reached..." << endl;
      //isGoalReached();
      //return true;
    } else {
      th_vel = AngularPIDController(base_odom, final_orientation[2], robot_curr_orien);
      cmd_vel.linear.x = 0;
      cmd_vel.linear.y = 0;
      cmd_vel.angular.z = th_vel;
      cout << "need to rotate on place... " << endl;
    }
  } else {  // quide the robot to the final goal position in the path
    //if (fabs(t_th_w - robot_curr_orien) <= o_precision_*2) {
    //********Linear velocity controller-PID*************
      x_vel = LinearPIDController(base_odom, t_x, t_y);
    //}
    //********Angular velocity controller-PID************
      if (plan_index_ >= global_plan_.size()-5) {
        t_th_w = final_orientation[2];
      }
      th_vel = AngularPIDController(base_odom, t_th_w, robot_curr_orien);
    //********************************************************************************
    cmd_vel.linear.x = x_vel;
    cmd_vel.linear.y = 0;
    cmd_vel.angular.z = th_vel;
 }

  // publish next target pose
  target.header.frame_id = "/map";
  target.header.stamp = ros::Time::now();
  target_pose_pub_.publish(target);

  // publish robot pose
  curr_pose.header.frame_id = "/map";
  curr_pose.header.stamp = ros::Time::now();
  tf::Quaternion curr_orien_quat =
  tf::createQuaternionFromYaw(robot_curr_orien);
  curr_pose.pose.position.x = robot_curr_pose[0];
  curr_pose.pose.position.y = robot_curr_pose[1];
  curr_pose.pose.position.z = robot_curr_pose[2];
  curr_pose.pose.orientation.x = curr_orien_quat[0];
  curr_pose.pose.orientation.y = curr_orien_quat[1];
  curr_pose.pose.orientation.z = curr_orien_quat[2];
  curr_pose.pose.orientation.w = curr_orien_quat[3];
  curr_pose_pub.publish(curr_pose);

  return true;
}

};  // namespace agribot_local_planner