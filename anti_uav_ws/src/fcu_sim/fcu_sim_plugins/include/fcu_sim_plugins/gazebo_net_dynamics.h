

#ifndef fcu_sim_PLUGINS_NET_DYNAMICS_H
#define fcu_sim_PLUGINS_NET_DYNAMICS_H

#include <stdio.h>

#include <boost/bind.hpp>
#include <Eigen/Eigen>
#include <tf/tf.h>

#include <gazebo/common/common.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <ros/callback_queue.h>
#include <ros/ros.h>

#include <fcu_common/Command.h>
#include <std_msgs/Float32.h>
#include <geometry_msgs/Vector3.h>

#include "fcu_sim_plugins/common.h"

using namespace Eigen;

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

  // double xddot;
  // double yddot;
  // double zddot;

  double phidot;
  double thetadot;
  double psidot;
}state_t;

namespace gazebo {
static const std::string kDefaultWindSpeedSubTopic = "gazebo/wind_speed";


class GazeboAircraftForcesAndMoments : public ModelPlugin {
 public:
  GazeboAircraftForcesAndMoments();

  ~GazeboAircraftForcesAndMoments();

  void InitializeParams();
  void SendForces();


 protected:
  void UpdateForcesAndMoments();
  void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf);
  void OnUpdate(const common::UpdateInfo & /*_info*/);

 private:
  std::string command_topic_;
  std::string wind_speed_topic_;
  std::string joint_name_;
  std::string link_name_;
  std::string parent_frame_id_;
  std::string motor_speed_pub_topic_;
  std::string namespace_;

  physics::WorldPtr world_;
  physics::ModelPtr model_;
  physics::LinkPtr link_;
  physics::JointPtr joint_;
  physics::EntityPtr parent_link_;
  event::ConnectionPtr updateConnection_; // Pointer to the update event connection.

  // physical parameters
  double agent_mass_;
  double net_mass_;
  double net_width_;

  state_t adj1_state_;
  state_t adj2_state_;
  state_t corner_state_;
  // state_t adj1_state_prev_;
  // state_t adj2_state_prev_;
  // state_t corner_state_prev_;

  // Vector3d adj1_accel_;
  // Vector3d adj2_accel_;
  // Vector3d corner_accel_;
  Vector3d model_forces_;
  Vector3d adj1_forces_;
  Vector3d adj2_forces_;
  Vector3d corner_forces_;

  Vector3d total_force_;


  // Time Counters
  double sampling_time_;
  double prev_sim_time_;

  ros::NodeHandle* node_handle_;
  ros::Subscriber command_sub_;
  ros::Subscriber wind_speed_sub_;
  ros::Subscriber adjacent1_pose_sub_;
  ros::Subscriber corner_pose_sub_;
  ros::Subscriber adjacent2_pose_sub_;

  ros::Subscriber model_forces_sub_;
  ros::Subscriber adjacent1_forces_sub_;
  ros::Subscriber corner_forces_sub_;
  ros::Subscriber adjacent2_forces_sub_;

  void CommandCallback(const fcu_common::CommandConstPtr& msg);
  void Adjacent1PoseCallback(const nav_msgs::OdometryConstPtr &msg);
  void Adjacent2PoseCallback(const nav_msgs::OdometryConstPtr &msg);
  void CornerPoseCallback(const nav_msgs::OdometryConstPtr &msg);
  void ModelForcesCallback(const geometry_msgs::Vector3ConstPtr &msg);
  void Adjacent1ForcesCallback(const geometry_msgs::Vector3ConstPtr &msg);
  void Adjacent2ForcesCallback(const geometry_msgs::Vector3ConstPtr &msg);
  void CornerForcesCallback(const geometry_msgs::Vector3ConstPtr &msg);

  // Utility functions
  Vector3d getForceOfNetBetweenAgents(state_t model_state, state_t other_state, Vector3d other_force, double edge_length);
  Vector3d rotateBodyToInertial(Vector3d body_vec, double phi, double theta, double psi);
  double tustinDerivative(double xdot, double x, double x_prev, double dt, double tau);
  Vector3d tustinDerivativeVector(Vector3d xdot, Vector3d x, Vector3d x_prev, double dt, double tau);
  Vector2d vectorProjection(Vector2d vec, Vector2d reference);

};
}

#endif // fcu_sim_PLUGINS_NET_DYNAMICS_H
