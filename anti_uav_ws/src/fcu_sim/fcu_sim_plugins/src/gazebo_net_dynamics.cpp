
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
  getSdfParam<std::string>(_sdf, "adjacent1PoseTopic", adj1_pose_topic, "wind");
  getSdfParam<std::string>(_sdf, "cornerPoseTopic", corner_pose_topic, "command");
  getSdfParam<std::string>(_sdf, "adjacent2PoseTopic", adj2_pose_topic, "command");

  // physical parameters
  getSdfParam<double>(_sdf, "agent_mass", agent_mass_, 10.0);
  getSdfParam<double>(_sdf, "net_mass", net_mass_, 2.0);


  // Connect the update function to the simulation
  updateConnection_ = event::Events::ConnectWorldUpdateBegin(boost::bind(&NetDynamics::OnUpdate, this, _1));

  // Connect Subscribers
  adjacent1_pose_sub_ = node_handle_->subscribe(adj1_pose_topic, 1, &NetDynamics::Adjacent1Callback, this);
  corner_pose_sub_ = node_handle_->subscribe(corner_pose_topic, 1, &NetDynamics::CornerCallback, this);
  adjacent2_pose_sub_ = node_handle_->subscribe(adj2_pose_topic, 1, &NetDynamics::Adjacent2Callback, this);
}

// This gets called by the world update event.
void NetDynamics::OnUpdate(const common::UpdateInfo& _info) {
  sampling_time_ = _info.simTime.Double() - prev_sim_time_;
  prev_sim_time_ = _info.simTime.Double();
  UpdateNetDynamics();
  SendForces();
}

void NetDynamics::Adjacent1Callback(const nav_msgs::OdometryConstPtr &msg)
{
  // TODO: extract state data
  
}


void NetDynamics::UpdateNetDynamics()
{
  // TODO: Check the distance between agents

  // Apply a force to agents who are too far away
  // Assume a simple spring model with fairly rigid elasticity

  // Calculate the amount of drag and weight of the net on the model based on relative positions
}


void NetDynamics::SendForces()
{
  // apply the forces and torques to the joint
  link_->AddRelativeForce(math::Vector3(forces_.Fx, -forces_.Fy, -forces_.Fz));
  link_->AddRelativeTorque(math::Vector3(forces_.l, -forces_.m, -forces_.n));
}

GZ_REGISTER_MODEL_PLUGIN(NetDynamics);
}
