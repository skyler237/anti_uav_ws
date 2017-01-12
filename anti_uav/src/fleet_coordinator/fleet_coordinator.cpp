#include "fleet_coordinator/fleet_coordinator.h"
#include <stdio.h>

namespace fleet_coordinator
{

FleetCoordinator::FleetCoordinator() :
  nh_(ros::NodeHandle()),
  nh_private_("~")
{

  // Parameters
  square_length_ = nh_private_.param<double>("square_length", 3.0); // [m]
  double x_init = nh_private_.param<double>("x_init", 0.0);
  double y_init = nh_private_.param<double>("y_init", 0.0);
  double z_init = nh_private_.param<double>("z_init", 0.0);
  double phi_init = nh_private_.param<double>("phi_init", 0.0);
  double theta_init = nh_private_.param<double>("theta_init", 0.0);
  double psi_init = nh_private_.param<double>("psi_init", 0.0);

  // Initialize position and rotation
  goal_xc_.pn = x_init;
  goal_xc_.pe = y_init;
  goal_xc_.pd = z_init;
  goal_xc_.phi = phi_init;
  goal_xc_.theta = theta_init;
  goal_xc_.psi = psi_init;

  // Set up Publishers and Subscriber
  goal_sub_ = nh_.subscribe("fleet_goal", 1000, &FleetCoordinator::goalCallback, this); // Position and orientation of the virtual rigid frame square

  uav1_goal_pub_ = nh_.advertise<geometry_msgs::Vector3>("uav1_goal", 1000);
  uav2_goal_pub_ = nh_.advertise<geometry_msgs::Vector3>("uav2_goal", 1000);
  uav3_goal_pub_ = nh_.advertise<geometry_msgs::Vector3>("uav3_goal", 1000);
  uav4_goal_pub_ = nh_.advertise<geometry_msgs::Vector3>("uav4_goal", 1000);

  uav1_goal_stamped_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("uav1_stamped_goal", 1000);
  uav2_goal_stamped_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("uav2_stamped_goal", 1000);
  uav3_goal_stamped_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("uav3_stamped_goal", 1000);
  uav4_goal_stamped_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("uav4_stamped_goal", 1000);

}

// void FleetCoordinator::goalCallback(const geometry_msgs::Vector3ConstPtr &msg)
// {
//   xc_.pn = msg->x;
//   xc_.pe = msg->y;
//   xc_.pd = msg->z;
// }

void FleetCoordinator::goalCallback(const nav_msgs::OdometryConstPtr &msg)
{
  // This should already be coming in NED
  goal_xc_.pn = msg->pose.pose.position.x;
  goal_xc_.pe = msg->pose.pose.position.y;
  goal_xc_.pd = msg->pose.pose.position.z;
  ROS_INFO("goal_xc_: x=%f, y=%f, z=%f", goal_xc_.pn, goal_xc_.pe, goal_xc_.pd);

  goal_xc_.u = msg->twist.twist.linear.x;
  goal_xc_.v = msg->twist.twist.linear.y;
  goal_xc_.w = msg->twist.twist.linear.z;

  // Convert Quaternion to RPY
  tf::Quaternion tf_quat;
  tf::quaternionMsgToTF(msg->pose.pose.orientation, tf_quat);
  tf::Matrix3x3(tf_quat).getRPY(goal_xc_.phi, goal_xc_.theta, goal_xc_.psi);
  goal_xc_.theta = goal_xc_.theta;
  goal_xc_.psi = goal_xc_.psi;

  goal_xc_.p = msg->twist.twist.angular.x;
  goal_xc_.q = msg->twist.twist.angular.y;
  goal_xc_.r = msg->twist.twist.angular.z;

  computeVertices();
  publishCommands();
}

void FleetCoordinator::computeVertices() {
  double now = ros::Time::now().toSec();
  double dt = now - prev_time_;
  prev_time_ = now;

  if(dt > 0.0)
  {

    // Initial distances in body frame
    uav1_xc_.pn = square_length_ / 2.0;    // NE
    uav1_xc_.pe = square_length_ / 2.0;
    uav1_xc_.pd = 0;
    ROS_INFO("uav1_xc_ initial: x=%f, y=%f, z=%f", uav1_xc_.pn, uav1_xc_.pe, uav1_xc_.pd);

    uav2_xc_.pn = -1.0*square_length_ / 2.0; // NW
    uav2_xc_.pe = square_length_ / 2.0;
    uav2_xc_.pd = 0;

    uav3_xc_.pn = -1.0*square_length_ / 2.0; // SW
    uav3_xc_.pe = -1.0*square_length_ / 2.0;
    uav3_xc_.pd = 0;

    uav4_xc_.pn = square_length_ / 2.0;    // SE
    uav4_xc_.pe = -1.0*square_length_ / 2.0;
    uav4_xc_.pd = 0;

    // Rotate vertices
    rotateVertex(&uav1_xc_, goal_xc_.phi, goal_xc_.theta, goal_xc_.psi);
    rotateVertex(&uav2_xc_, goal_xc_.phi, goal_xc_.theta, goal_xc_.psi);
    rotateVertex(&uav3_xc_, goal_xc_.phi, goal_xc_.theta, goal_xc_.psi);
    rotateVertex(&uav4_xc_, goal_xc_.phi, goal_xc_.theta, goal_xc_.psi);

    ROS_INFO("uav1_xc_ rotated: x=%f, y=%f, z=%f", uav1_xc_.pn, uav1_xc_.pe, uav1_xc_.pd);

    // Translate to correct position
    translateVertex(&uav1_xc_, goal_xc_.pn, goal_xc_.pe, goal_xc_.pd);
    translateVertex(&uav2_xc_, goal_xc_.pn, goal_xc_.pe, goal_xc_.pd);
    translateVertex(&uav3_xc_, goal_xc_.pn, goal_xc_.pe, goal_xc_.pd);
    translateVertex(&uav4_xc_, goal_xc_.pn, goal_xc_.pe, goal_xc_.pd);
    ROS_INFO("uav1_xc_ translated: x=%f, y=%f, z=%f", uav1_xc_.pn, uav1_xc_.pe, uav1_xc_.pd);

    uav1_command_ = convertToCommand(uav1_xc_);
    uav2_command_ = convertToCommand(uav2_xc_);
    uav3_command_ = convertToCommand(uav3_xc_);
    uav4_command_ = convertToCommand(uav4_xc_);
  }
}

void FleetCoordinator::rotateVertex(state_t* vertex, double phi, double theta, double psi) {
  // Performs roll, pitch, then yaw rotations
  double x = vertex->pn;
  double y = vertex->pe;
  double z = vertex->pd;
  vertex->pn = (cos(theta)*cos(psi))*x + (cos(theta)*sin(psi))*y - sin(theta)*z;
  vertex->pe = (sin(phi)*sin(theta)*cos(psi) - cos(phi)*sin(psi))*x + (sin(phi)*sin(theta)*sin(psi) + cos(phi)*cos(psi))*y + sin(phi)*cos(theta)*z;
  vertex->pd = (cos(phi)*sin(theta)*cos(psi) + sin(phi)*sin(psi))*x + (cos(phi)*sin(theta)*sin(psi) - sin(phi)*cos(psi))*y + cos(phi)*cos(theta)*z;
}

void FleetCoordinator::translateVertex(state_t* vertex, double x, double y, double z) {
  // Translates in x, y, and z
  vertex->pn = vertex->pn + x;
  vertex->pe = vertex->pe + y;
  vertex->pd = vertex->pd + z;
}

geometry_msgs::Vector3 FleetCoordinator::convertToCommand(state_t state) {
  geometry_msgs::Vector3 command;
  command.x = state.pn;
  command.y = state.pe;
  command.z = state.pd;

  return command;
}

void FleetCoordinator::publishCommands()
{
  uav1_goal_pub_.publish(uav1_command_);
  // ROS_INFO("UAV1 Goal: x=%f, y=%f, z=%f", uav1_command_.x, uav1_command_.y, uav1_command_.z);
  uav2_goal_pub_.publish(uav2_command_);
  // ROS_INFO("UAV2 Goal: x=%f, y=%f, z=%f", uav2_command_.x, uav2_command_.y, uav2_command_.z);
  uav3_goal_pub_.publish(uav3_command_);
  // ROS_INFO("UAV3 Goal: x=%f, y=%f, z=%f", uav3_command_.x, uav3_command_.y, uav3_command_.z);
  uav4_goal_pub_.publish(uav4_command_);
  // ROS_INFO("UAV4 Goal: x=%f, y=%f, z=%f", uav4_command_.x, uav4_command_.y, uav4_command_.z);


  // Publish a stamped version also (the controller expects a Vector3, but for data analysis, a stamped topic is more useful)
  geometry_msgs::PoseStamped uav1_stamped_command_;
  uav1_stamped_command_.pose.position.x = uav1_command_.x;
  uav1_stamped_command_.pose.position.y = uav1_command_.y;
  uav1_stamped_command_.pose.position.z = uav1_command_.z;

  geometry_msgs::PoseStamped uav2_stamped_command_;
  uav2_stamped_command_.pose.position.y = uav2_command_.y;
  uav2_stamped_command_.pose.position.x = uav2_command_.x;
  uav2_stamped_command_.pose.position.z = uav2_command_.z;

  geometry_msgs::PoseStamped uav3_stamped_command_;
  uav3_stamped_command_.pose.position.y = uav3_command_.y;
  uav3_stamped_command_.pose.position.x = uav3_command_.x;
  uav3_stamped_command_.pose.position.z = uav3_command_.z;

  geometry_msgs::PoseStamped uav4_stamped_command_;
  uav4_stamped_command_.pose.position.y = uav4_command_.y;
  uav4_stamped_command_.pose.position.x = uav4_command_.x;
  uav4_stamped_command_.pose.position.z = uav4_command_.z;

  uav1_goal_stamped_pub_.publish(uav1_stamped_command_);
  uav2_goal_stamped_pub_.publish(uav2_stamped_command_);
  uav3_goal_stamped_pub_.publish(uav3_stamped_command_);
  uav4_goal_stamped_pub_.publish(uav4_stamped_command_);

}

} // namespace fleet_coordinator
