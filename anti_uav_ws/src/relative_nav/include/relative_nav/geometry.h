/*! \file eigen.h
  * \author David Wheeler
  * \date June 2014
  *
  * \brief Useful functions for dealing with eigen matrices.
  *
*/

#ifndef relative_nav_GEOMETRY_H
#define relative_nav_GEOMETRY_H

#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Geometry"
#include "ros/ros.h"
#include <tf/tf.h>
#include <geometry_msgs/Quaternion.h>

namespace relative_nav
{

void roll_pitch_yaw_from_quaternion(Eigen::Matrix<double,4,1> q, double &phi, double &theta, double &psi);
void roll_pitch_yaw_from_quaternion(Eigen::Quaterniond q, double &phi, double &theta, double &psi);
void roll_pitch_yaw_from_quaternion(tf::Quaternion q, double &phi, double &theta, double &psi);
void roll_pitch_yaw_from_quaternion(geometry_msgs::Quaternion q, double &phi, double &theta, double &psi);

Eigen::Matrix<double,4,1> quaternion_from_roll_pitch_yaw(double phi, double theta, double psi);
void quaternion_from_roll_pitch_yaw(double phi, double theta, double psi, tf::Quaternion& q);

Eigen::Matrix3d rotation_from_roll_pitch_yaw(double roll, double pitch, double yaw);

/*!
  \brief Returns the rotation matrix that would rotate the frame that a vector is expressed in according to
          the quaternion (Hamiltonian notation).
  \param q is 4x1 (with the order qx,qy,qz,qw)
  */
Eigen::Matrix3d rotate_frame_using_quaternion(double qx, double qy, double qz, double qo);
/*!
  \brief Returns the rotation matrix that would rotate the frame that a vector is expressed in according to
          the quaternion (Hamiltonian notation).
  \param q is 4x1 (with the order qx,qy,qz,qw)
  */
Eigen::Matrix3d rotate_frame_using_quaternion(Eigen::Matrix<double,4,1> q);

Eigen::Matrix3d skew(Eigen::Vector3d vec);

/*!
  \brief Quaternion Kinematic Matrix, OMEGA(omega)
  \param pqr is angular rates p, q, and r

  qdot is represented as 0.5*omega(pqr)*q
  Assumes q is 4x1 (with the order qx,qy,qz,qw)
  */
Eigen::Matrix<double,4,4> Omega(Eigen::Vector3d pqr);

Eigen::Matrix<double,4,1> quaternionMultiplication(Eigen::Matrix<double,4,1> p,Eigen::Matrix<double,4,1> q);
Eigen::Matrix<double,4,1> quaternionMultiplicationJPL(Eigen::Matrix<double,4,1> p,Eigen::Matrix<double,4,1> q);

Eigen::Matrix4d conjugate();

Eigen::Matrix3d M(double mass);

void cameraParamsToTransform(const ros::NodeHandle &nh, tf::Transform &camera_to_body);

} //end namespace

#endif
