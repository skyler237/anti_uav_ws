#ifndef INTERCEPT_CONTROLLER_H
#define INTERCEPT_CONTROLLER_H

#include <ros/ros.h>
#include <fcu_common/ExtendedCommand.h>
#include <anti_uav/PathCoeff.h>
#include <fcu_common/simple_pid.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>
#include <tf/tf.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <anti_uav/InterceptResult.h>

#include <dynamic_reconfigure/server.h>
#include <anti_uav/InterceptControllerConfig.h>

#define MAX_WAYPOINT_CNT 20

namespace intercept_controller
{

// typedef struct
// {
//   double pn;
//   double pe;
//   double pd;
//
//   double phi;
//   double theta;
//   double psi;
//
//   double u;
//   double v;
//   double w;
//
//   double p;
//   double q;
//   double r;
// }state_t;

typedef boost::shared_ptr< ::anti_uav::InterceptResult const> InterceptResultConstPtr;

typedef struct
{
  double x;
  double y;
  double z;

  double phi;
  double theta;
  double psi;

  double xdot;
  double ydot;
  double zdot;

  double phidot;
  double thetadot;
  double psidot;
}state_t;


// typedef struct
// {
//   double roll;
//   double pitch;
//   double yaw_rate;
//   double throttle;
//   double u;
//   double v;
// } max_t;

struct Max {
  double roll;
  double pitch;
  double phi_rate;
  double theta_rate;
  double psi_rate;
  double x_rate;
  double y_rate;
  double z_rate;
  double phidot_rate;
  double thetadot_rate;
  double psidot_rate;
  double xdot_rate;
  double ydot_rate;
  double zdot_rate;
  double velocity;
  double acceleration;

  double pos_error;
  int waypoint_cnt;
  double control_delay;
};

struct Path_t {
  Eigen::Vector3d waypoints[MAX_WAYPOINT_CNT];
  int waypoint_cnt;
  double path_length;
};

class InterceptController
{

public:

  InterceptController();

private:

  // Node handles, publishers, subscribers
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  // Publishers and Subscribers
  ros::Subscriber state_sub_;
  ros::Subscriber is_flying_sub_;
  ros::Subscriber target_sub_;
  ros::Subscriber uav1_state_sub_;
  ros::Subscriber uav2_state_sub_;
  ros::Subscriber uav3_state_sub_;
  ros::Subscriber uav4_state_sub_;
  ros::Subscriber sim_results_sub_;


  ros::Publisher fleet_goal_pub_;
  ros::Publisher path_coeff_pub_;

  // Paramters
  double thrust_eq_;
  bool is_flying_;
  double distance_start_rotation_;
  double distance_finish_rotation_;
  double fleet_square_width_;

  // Memory for sharing information between functions
  state_t xhat_; // estimate
  Max max_;
  nav_msgs::Odometry command_msg_;
  anti_uav::PathCoeff debug_msg_;
  state_t xt_; // target
  state_t fleet_goal_; // path for fleet to follow
  state_t uav1_state_;
  state_t uav2_state_;
  state_t uav3_state_;
  state_t uav4_state_;
  state_t fleet_state_; // Represents the actual average state of the fleet
  double prev_time_;
  double protection_radius_;
  double current_radius_;
  double max_radius_;
  double min_radius_;
  double full_response_radius_;
  double alpha0_;

  Path_t path_;
  int waypoint_index_ = 0;

  bool resetting_ = false;


  // Functions
  void stateCallback(const nav_msgs::OdometryConstPtr &msg);
  void isFlyingCallback(const std_msgs::BoolConstPtr &msg);
  // void targetCallback(const geometry_msgs::Vector3ConstPtr &msg);
  void targetCallback(const nav_msgs::OdometryConstPtr &msg);
  void uav1_stateCallback(const nav_msgs::OdometryConstPtr &msg);
  void uav2_stateCallback(const nav_msgs::OdometryConstPtr &msg);
  void uav3_stateCallback(const nav_msgs::OdometryConstPtr &msg);
  void uav4_stateCallback(const nav_msgs::OdometryConstPtr &msg);
  void sim_resultsCallback(const anti_uav::InterceptResultConstPtr &msg);

  // Helper functions
  Eigen::Vector3d targetPredict(double time);
  Path_t getSimpleWaypointPath(Eigen::Vector3d fleet_pos, Eigen::Vector3d target_pos, Eigen::Vector3d v_final);
  Path_t getSmoothedWaypointPath(Eigen::Vector3d fleet_pos, Eigen::Vector3d target_pos, Eigen::Vector3d v_final);
  Path_t getProtectionRadiusWaypointPath(Eigen::Vector3d fleet_pos, Eigen::Vector3d target_pos, Eigen::Vector3d v_final);
  Path_t getAdaptiveProtectionRadiusWaypointPath(Eigen::Vector3d fleet_pos, Eigen::Vector3d target_pos) ;
  Path_t planPath(Eigen::Vector3d fleet_pos, Eigen::Vector3d target_pos, Eigen::Vector3d v_final);
  double computeInterceptTime();
  double time_delta(double v, double T, Eigen::Vector3d z, Eigen::Vector3d v_intercept);
  double pathLength(Path_t* path);
  void computeFleetState();

  void computeControl();
  void computeProNavControl();
  void resetStates();
  void publishCommand();
  double saturate(double x, double max, double min);
  Eigen::Vector3d saturate_vector(Eigen::Vector3d vector, double max, double min);
  double sgn(double x);
  double min(double x, double y);
  double tustinDerivative(double xdot, double x, double x_prev, double dt, double tau);
};
}

#endif
