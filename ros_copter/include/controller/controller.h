#ifndef CONTROLLER_H
#define CONTROLLER_H

#include <ros/ros.h>
#include <fcu_common/ExtendedCommand.h>
#include <fcu_common/simple_pid.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Bool.h>
#include <tf/tf.h>

#include <dynamic_reconfigure/server.h>
#include <ros_copter/ControllerConfig.h>

namespace controller
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

typedef struct
{
  double roll;
  double pitch;
  double yaw_rate;
  double throttle;
  double u;
  double v;
} max_t;

class Controller
{

public:

  Controller();

private:

  // Node handles, publishers, subscribers
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  // Publishers and Subscribers
  ros::Subscriber state_sub_;
  ros::Subscriber is_flying_sub_;
  ros::Subscriber goal_sub_;

  ros::Publisher command_pub_;

  // Paramters
  int model_number_;
  double thrust_eq_;
  bool is_flying_;
  //bool always_flying_; // Can override the is_flying message

  // PID Controllers
  fcu_common::SimplePID PID_u_;
  fcu_common::SimplePID PID_v_;
  fcu_common::SimplePID PID_x_;
  fcu_common::SimplePID PID_y_;
  fcu_common::SimplePID PID_z_;
  fcu_common::SimplePID PID_psi_;

  // Dynamic Reconfigure Hooks
  dynamic_reconfigure::Server<ros_copter::ControllerConfig> _server;
  dynamic_reconfigure::Server<ros_copter::ControllerConfig>::CallbackType _func;
  void reconfigure_callback(ros_copter::ControllerConfig &config, uint32_t level);

  // Memory for sharing information between functions
  state_t xhat_; // estimate
  max_t max_;
  fcu_common::ExtendedCommand command_;
  state_t xc_; // command
  double prev_time_;


  // Functions
  void stateCallback(const nav_msgs::OdometryConstPtr &msg);
  void isFlyingCallback(const std_msgs::BoolConstPtr &msg);
  void goalCallback(const geometry_msgs::Vector3ConstPtr &msg);

  void computeControl();
  void resetIntegrators();
  void publishCommand();
  double saturate(double x, double max, double min);
  double sgn(double x);
};
}

#endif
