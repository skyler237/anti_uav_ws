#ifndef SIM_ANALYSIS_H
#define SIM_ANALYSIS_H

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <anti_uav/InterceptResult.h>
#include <geometry_msgs/Vector3.h>
#include <nav_msgs/Odometry.h>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include "gazebo_msgs/ModelState.h"
#include "gazebo/physics/physics.hh"


namespace sim_analysis
{

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


class SimAnalysis
{

public:

  SimAnalysis();

private:

  // Node handles, publishers, subscribers
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  // Publishers and Subscribers
  ros::Subscriber intruder_state_sub_;
  ros::Subscriber uav1_state_sub_;
  ros::Subscriber uav2_state_sub_;
  ros::Subscriber uav3_state_sub_;
  ros::Subscriber uav4_state_sub_;

  ros::Publisher results_pub_;

  // Paramters
  double net_width_;

  // Memory for sharing information between functions

  state_t uav1_state_;
  state_t uav2_state_;
  state_t uav3_state_;
  state_t uav4_state_;
  state_t intruder_state_; // Represents the actual average state of the fleet
  anti_uav::InterceptResult result_msg_;

  geometry_msgs::Pose fleet_reset_pose_;
  geometry_msgs::Pose intruder_reset_pose_;
  geometry_msgs::Pose uav1_reset_pose_;
  geometry_msgs::Pose uav2_reset_pose_;
  geometry_msgs::Pose uav3_reset_pose_;
  geometry_msgs::Pose uav4_reset_pose_;
  geometry_msgs::Twist reset_twist_;

  std::string world_name_;
  std::string intruder_name_;
  std::string uav1_name_;
  std::string uav2_name_;
  std::string uav3_name_;
  std::string uav4_name_;

  double last_reset_;
  double prev_time_;


  // Functions
  void intruder_stateCallback(const nav_msgs::OdometryConstPtr &msg);
  void uav1_stateCallback(const nav_msgs::OdometryConstPtr &msg);
  void uav2_stateCallback(const nav_msgs::OdometryConstPtr &msg);
  void uav3_stateCallback(const nav_msgs::OdometryConstPtr &msg);
  void uav4_stateCallback(const nav_msgs::OdometryConstPtr &msg);


  void publishResults();
  void checkIntercept();
  void resetSimulation();
};
}

#endif
