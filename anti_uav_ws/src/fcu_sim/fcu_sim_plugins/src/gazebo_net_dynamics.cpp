
#include "fcu_sim_plugins/gazebo_net_dynamics.h"

namespace gazebo
{

NetDynamics::NetDynamics() :
  ModelPlugin(), node_handle_(nullptr), prev_sim_time_(0)  {}


NetDynamics::~NetDynamics()
{
  event::Events::DisconnectWorldUpdateBegin(updateConnection_);
  if (node_handle_) {
    node_handle_->shutdown();
    delete node_handle_;
  }
}

void NetDynamics::Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  model_ = _model;
  world_ = model_->GetWorld();
  namespace_.clear();

  /*
   * Connect the Plugin to the Robot and Save pointers to the various elements in the simulation
   */
  if (_sdf->HasElement("namespace"))
    namespace_ = _sdf->GetElement("namespace")->Get<std::string>();
  else
    gzerr << "[gazebo_net_dynamics] Please specify a namespace.\n";
  node_handle_ = new ros::NodeHandle(namespace_);

 if (_sdf->HasElement("linkName"))
    link_name_ = _sdf->GetElement("linkName")->Get<std::string>();
  else
    gzerr << "[gazebo_net_dynamics] Please specify a linkName of the forces and moments plugin.\n";
  link_ = model_->GetLink(link_name_);
  if (link_ == NULL)
    gzthrow("[gazebo_net_dynamics] Couldn't find specified link \"" << link_name_ << "\".");

  /* Load Params from Gazebo Server */
  getSdfParam<std::string>(_sdf, "adjacent1PoseTopic", adj1_pose_topic, "adj1_pose");
  getSdfParam<std::string>(_sdf, "cornerPoseTopic", corner_pose_topic, "corner_pose");
  getSdfParam<std::string>(_sdf, "adjacent2PoseTopic", adj2_pose_topic, "adj2_pose");

  // physical parameters
  getSdfParam<double>(_sdf, "agent_mass", agent_mass_, 10.0);
  getSdfParam<double>(_sdf, "net_mass", net_mass_, 2.0);
  getSdfParam<double>(_sdf, "net_width", net_width_, 3.0); // [m]
  getSdfParam<double>(_sdf, "dirty_deriv_gain", tau_, 0.1);


  // Connect the update function to the simulation
  updateConnection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&NetDynamics::OnUpdate, this, _1));

  // Connect Subscribers
  adjacent1_pose_sub_ = node_handle_->subscribe(adj1_pose_topic, 1, &NetDynamics::Adjacent1PoseCallback, this);
  corner_pose_sub_ = node_handle_->subscribe(corner_pose_topic, 1, &NetDynamics::CornerPoseCallback, this);
  adjacent2_pose_sub_ = node_handle_->subscribe(adj2_pose_topic, 1, &NetDynamics::Adjacent2PoseCallback, this);

  model_forces_sub_ = node_handle_->subscribe(model_forces_topic, 1, &NetDynamics::ModelForcesCallback, this);
  adjacent1_forces_sub_ = node_handle_->subscribe(adj1_forces_topic, 1, &NetDynamics::Adjacent1ForcesCallback, this);
  corner_forces_sub_ = node_handle_->subscribe(corner_forces_topic, 1, &NetDynamics::CornerForcesCallback, this);
  adjacent2_forces_sub_ = node_handle_->subscribe(adj2_forces_topic, 1, &NetDynamics::Adjacent2ForcesCallback, this);
}

// This gets called by the world update event.
void NetDynamics::OnUpdate(const common::UpdateInfo& _info) {
  sampling_time_ = _info.simTime.Double() - prev_sim_time_;
  prev_sim_time_ = _info.simTime.Double();
  UpdateNetDynamics();
  SendForces();
}

// state_t NetDynamics::extractStateInfo(const nav_msgs::OdometryConstPtr &msg, state_t prev_state, double dt)
state_t NetDynamics::extractStateInfo(const nav_msgs::OdometryConstPtr &msg)
{
  state_t state;
  // Convert Quaternion to RPY
  tf::Quaternion tf_quat;
  tf::quaternionMsgToTF(msg->pose.pose.orientation, tf_quat);
  tf::Matrix3x3(tf_quat).getRPY(state.phi, state.theta, state.psi);

  state.phidot = msg->twist.twist.angular.x;
  state.thetadot = msg->twist.twist.angular.x;
  state.psidot = msg->twist.twist.angular.x;

  state.x = msg->pose.pose.position.x;
  state.y = msg->pose.pose.position.y;
  state.z = msg->pose.pose.position.z;

  Vector3d body_vel(msg->twist.twist.linear.x, msg->twist.twist.linear.y, msg->twist.twist.linear.z);
  Vector3d inertial_vel = rotateBodyToInertial(body_vel, state.phi, state.theta, state.psi);
  // Vector3d vel_prev(prev_state.xdot, prev_state.ydot, prev_state.zdot);
  // Vector3d accel_prev(prev_state.xddot, prev_state.yddot, prev_state.zddot);
  //
  // Vector3d inertial_accel = tustinDerivativeVector(accel_prev, intertial_vel, vel_prev, dt, tau_);

  state.xdot = inertial_vel(0);
  state.ydot = inertial_vel(1);
  state.zdot = inertial_vel(2);
  // state.xddot = inertial_accel(0);
  // state.yddot = inertial_accel(1);
  // state.zddot = inertial_accel(2);

  return state;
}

void NetDynamics::Adjacent1PoseCallback(const nav_msgs::OdometryConstPtr &msg) {
  // static double prev_time = msg->header.stamp.toSec();
  // double now = msg->header.stamp.toSec();
  // double dt = now - prev_time;
  // prev_time = now;
  adj1_state_ = extractStateInfo(msg, adj1_state_prev_, dt);
  // adj1_state_prev_ = adj1_state_;
}

void NetDynamics::Adjacent2PoseCallback(const nav_msgs::OdometryConstPtr &msg) {
  // static double prev_time = msg->header.stamp.toSec();
  // double now = msg->header.stamp.toSec();
  // double dt = now - prev_time;
  // prev_time = now;
  adj2_state_ = extractStateInfo(msg, adj2_state_prev_, dt);
  // adj2_state_prev_ = adj2_state_;
}

void NetDynamics::CornerPoseCallback(const nav_msgs::OdometryConstPtr &msg) {
  // static double prev_time = msg->header.stamp.toSec();
  // double now = msg->header.stamp.toSec();
  // double dt = now - prev_time;
  // prev_time = now;
  corner_state_ = extractStateInfo(msg, corner_state_prev_, dt);
  // corner_state_prev_ = corner_state_;
}

void NetDynamics::ModelForcesCallback(const geometry_msgs::Vector3ConstPtr &msg) {
  model_forces_.x = msg->x;
  model_forces_.y = msg->y;
  model_forces_.z = msg->z;
}

void NetDynamics::Adjacent1ForcesCallback(const geometry_msgs::Vector3ConstPtr &msg) {
  adj1_forces_.x = msg->x;
  adj1_forces_.y = msg->y;
  adj1_forces_.z = msg->z;
}

void NetDynamics::Adjacent2ForcesCallback(const geometry_msgs::Vector3ConstPtr &msg) {
  adj2_forces_.x = msg->x;
  adj2_forces_.y = msg->y;
  adj2_forces_.z = msg->z;
}

void NetDynamics::ModelForcesCallback(const geometry_msgs::Vector3ConstPtr &msg) {
  corner_forces_.x = msg->x;
  corner_forces_.y = msg->y;
  corner_forces_.z = msg->z;
}


void NetDynamics::UpdateNetDynamics()
{
  // Get model's pose
  math::Pose model_pose = link_->GetWorldCoGPose();
  state_t model_state;
  // Convert to NED
  model_state.x = model_pose.pos.x;
  model_state.y = -model_pose.pos.y;
  model_state.z = -model_pose.pos.z;

  // Apply a force to agents who are too far away
  // Assume a simple spring model with fairly rigid elasticity
  Vector3d adj1_force = getForceOfNetBetweenAgents(model_state, adj1_state_, net_width_);
  Vector3d adj2_force = getForceOfNetBetweenAgents(model_state, adj2_state_, net_width_);
  Vector3d corner_force = getForceOfNetBetweenAgents(model_state, corner_state_, net_width_*sqrt(2.0));

  total_force_ = adj1_force + adj2_force + corner_force;

  // Calculate the amount of drag and weight of the net on the model based on relative positions
}

Vector3d NetDynamics::getForceOfNetBetweenAgents(state_t model_state, state_t other_state, Vector3d other_force, double edge_length) {
  Vector3d force(0.0, 0.0, 0.0);
  Vector3d model2other(other_state.x - model_state.x, other_state.y - model_state.y, other_state.z - model_state.z);
  double distance = model2other.norm();

  if (distance >= edge_length) {
    // Project both forces onto the line of sight vector between them
    model_proj_force = vectorProjection(model_forces_, model2other);
    other_proj_force = vectorProjection(other_force, model2other);

    // Find tensile force
    force = (other_proj_force - model_proj_force)/2.0;
  }

  return force;
}


void NetDynamics::SendForces()
{
  // apply the forces and torques to the joint
  link_->AddRelativeForce(math::Vector3(total_force_.x, -total_force_.y, -total_force_.z));
}

Vector3d NetDynamics::rotateBodyToInertial(Vector3d body_vec, double phi, double theta, double psi) {
  Vector3d inertial_vec;

  double cp = cos(phi);
  double sp = sin(phi);
  double ct = cos(theta);
  double st = sin(theta);
  double cs = cos(psi);
  double ss = sin(psi);

  Matrix3x3d R;
  R <<  ct*cs, sp*st*cs - cp*ss, cp*st*cs + sp*ss,
        ct*ss, sp*st*ss + cp*cs, cp*st*ss - sp*cs,
        -st,          sp*ct,            cp*ct;

  inertial_vec = R*body_vec;

  return inertial_vec;
}

double NetDynamics::tustinDerivative(double xdot, double x, double x_prev, double dt, double tau) {
  return (2.0*tau - dt)/(2.0*tau + dt)*xdot + 2.0/(2.0*tau + dt)*(x - x_prev);
}

Vector3d NetDynamics::tustinDerivativeVector(Vector3d xdot, Vector3d x, Vector3d x_prev, double dt, double tau) {
  Vector3d derivative;
  derivative(0) = tustinDerivative(xdot(0), x(0), x_prev(0), dt, tau);
  derivative(1) = tustinDerivative(xdot(1), x(1), x_prev(1), dt, tau);
  derivative(2) = tustinDerivative(xdot(2), x(2), x_prev(2), dt, tau);
  return derivative;
}

Vector3d NetDynamics::vectorProjection(Vector3d vec, Vector3d reference)
{
  if(reference.norm() != 0) {
    return vec.dot(reference)/reference.squaredNorm()*reference;
  }
  else {
    return Vector3d(0, 0, 0);
  }
}

GZ_REGISTER_MODEL_PLUGIN(NetDynamics);
}
