#ifndef FLEET_COORDINATOR_H
#define FLEET_COORDINATOR_H

#include <ros/ros.h>
#include <fcu_common/ExtendedCommand.h>
#include <fcu_common/simple_pid.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>
#include <tf/tf.h>

#include <dynamic_reconfigure/server.h>
#include <anti_uav/FleetCoordinatorConfig.h>

namespace fleet_coordinator
{

typedef struct
{
  double pn;
  double pe;
  double pd;

  double phi;
  double theta;
  double psi;

  double u;
  double v;
  double w;

  double p;
  double q;
  double r;
}state_t;

class FleetCoordinator
{

public:

  FleetCoordinator();

private:

  // Node handles, publishers, subscribers
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  // Publishers and Subscribers
  ros::Subscriber goal_sub_;

  ros::Publisher uav1_goal_pub_;
  ros::Publisher uav2_goal_pub_;
  ros::Publisher uav3_goal_pub_;
  ros::Publisher uav4_goal_pub_;

  ros::Publisher uav1_goal_stamped_pub_;
  ros::Publisher uav2_goal_stamped_pub_;
  ros::Publisher uav3_goal_stamped_pub_;
  ros::Publisher uav4_goal_stamped_pub_;

  // Paramters
  double square_length_;

  // Memory for sharing information between functions
  state_t goal_xc_; // Center desired state

  state_t uav1_xc_; // Distributed individual desired states
  state_t uav2_xc_;
  state_t uav3_xc_;
  state_t uav4_xc_;
  geometry_msgs::Vector3 uav1_command_;
  geometry_msgs::Vector3 uav2_command_;
  geometry_msgs::Vector3 uav3_command_;
  geometry_msgs::Vector3 uav4_command_;
  geometry_msgs::PoseStamped uav1_stamped_command_;
  geometry_msgs::PoseStamped uav2_stamped_command_;
  geometry_msgs::PoseStamped uav3_stamped_command_;
  geometry_msgs::PoseStamped uav4_stamped_command_;

  double prev_time_;


  // Functions
  // void goalCallback(const geometry_msgs::PoseStampedConstPtr &msg);
  void goalCallback(const nav_msgs::OdometryConstPtr &msg);


  void computeVertices();
  void publishCommands();

  void rotateVertex(state_t* vertex, double phi, double theta, double psi);
  void translateVertex(state_t* vertex, double x, double y, double z);
  geometry_msgs::Vector3 convertToCommand(state_t state);
};
}

#endif
