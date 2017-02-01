  #include "relative_nav/geometry.h"

namespace relative_nav
{

void roll_pitch_yaw_from_quaternion(Eigen::Matrix<double,4,1> q, double &phi, double &theta, double &psi)
{
  double ex = q(0,0);
  double ey = q(1,0);
  double ez = q(2,0);
  double eo = q(3,0);
  phi   = atan2(2.0*eo*ex + 2.0*ey*ez, eo*eo + ez*ez - ex*ex - ey*ey);
  theta = asin(2.0*eo*ey - 2.0*ex*ez);
  psi   = atan2(2.0*eo*ez + 2.0*ex*ey, eo*eo + ex*ex - ey*ey - ez*ez);
}

void roll_pitch_yaw_from_quaternion(Eigen::Quaterniond q, double &phi, double &theta, double &psi)
{
    Eigen::Matrix<double,4,1> q_mat(q.x(), q.y(), q.z(), q.w());
    roll_pitch_yaw_from_quaternion(q_mat, phi, theta, psi);
}

void roll_pitch_yaw_from_quaternion(tf::Quaternion q, double &phi, double &theta, double &psi)
{
    Eigen::Matrix<double,4,1> q_mat(q.x(), q.y(), q.z(), q.w());
    roll_pitch_yaw_from_quaternion(q_mat, phi, theta, psi);
}

void roll_pitch_yaw_from_quaternion(geometry_msgs::Quaternion q, double &phi, double &theta, double &psi)
{
  Eigen::Matrix<double,4,1> q_mat(q.x, q.y, q.z, q.w);
  roll_pitch_yaw_from_quaternion(q_mat, phi, theta, psi);
}

void quaternion_from_roll_pitch_yaw(double phi, double theta, double psi, tf::Quaternion& q)
{
  double sp = sin(phi/2);
  double cp = cos(phi/2);
  double st = sin(theta/2);
  double ct = cos(theta/2);
  double ss = sin(psi/2);
  double cs = cos(psi/2);

  q.setX(sp*ct*cs - cp*st*ss);
  q.setY(cp*st*cs + sp*ct*ss);
  q.setZ(cp*ct*ss - sp*st*cs);
  q.setW(cp*ct*cs + sp*st*ss);
}

Eigen::Matrix<double,4,1> quaternion_from_roll_pitch_yaw(double phi, double theta, double psi)
{
  double sp = sin(phi/2);
  double cp = cos(phi/2);
  double st = sin(theta/2);
  double ct = cos(theta/2);
  double ss = sin(psi/2);
  double cs = cos(psi/2);

  Eigen::Matrix<double,4,1> q;
  q << sp*ct*cs - cp*st*ss, \
       cp*st*cs + sp*ct*ss, \
       cp*ct*ss - sp*st*cs, \
       cp*ct*cs + sp*st*ss;
  return q;
}


Eigen::Matrix3d rotation_from_roll_pitch_yaw(double phi, double theta, double psi)
{
  Eigen::Matrix3d R,R1,R2,R3;
  // Yaw Rotate
  R1 << cos(psi), sin(psi), 0, \
      -sin(psi), cos(psi), 0, \
      0,        0,        1;
  // Pitch Rotate
  R2 << cos(theta), 0, -sin(theta), \
      0, 1, 0,           \
      sin(theta), 0, cos(theta);
  // Roll Rotate
  R3 << 1,         0,         0, \
      0,  cos(phi),  sin(phi), \
      0, -sin(phi),  cos(phi);
  R = R3*R2*R1;
  return R;
}

Eigen::Matrix3d rotate_frame_using_quaternion(double qx, double qy, double qz, double qo)
{
  Eigen::Matrix3d R_body_to_vehicle;
  R_body_to_vehicle << qo*qo + qx*qx - qy*qy - qz*qz,         2*qo*qz + 2*qx*qy,         2*qx*qz - 2*qo*qy, \
                               2*qx*qy - 2*qo*qz, qo*qo - qx*qx + qy*qy - qz*qz,         2*qo*qx + 2*qy*qz, \
                               2*qo*qy + 2*qx*qz,         2*qy*qz - 2*qo*qx, qo*qo - qx*qx - qy*qy + qz*qz;
  return R_body_to_vehicle;
}

/*!
  \brief Returns the body_to_vehicle transformation represented as rotation matrix.
  \param q is 4x1 (with the order qx,qy,qz,qw)
  */
Eigen::Matrix3d rotate_frame_using_quaternion(Eigen::Matrix<double,4,1> q)
{
  return rotate_frame_using_quaternion(q(0), q(1), q(2), q(3));
}

inline Eigen::Matrix3d skew(Eigen::Vector3d vec)
{
  Eigen::Matrix3d SS;
  SS << 0,     -vec(2), vec(1), \
      vec(2), 0,     -vec(0), \
      -vec(1), vec(0), 0;
  return SS;
}

/*!
  \brief Quaternion Kinematic Matrix, OMEGA(omega)
  \param pqr is angular rates p, q, and r

  qdot is represented as 0.5*omega(pqr)*q
  Assumes q is 4x1 (with the order qx,qy,qz,qw)
  */
Eigen::Matrix<double,4,4> Omega(Eigen::Vector3d pqr)
{
  Eigen::Matrix<double,4,4> omega;
  omega.block<3,3>(0,0) = -skew(pqr);
  omega.block<3,1>(0,3) = pqr;
  omega.block<1,3>(3,0) = -pqr.transpose();
  omega(3,3) = 0;
  return omega;
}

Eigen::Matrix<double,4,1> quaternionMultiplication(Eigen::Matrix<double,4,1> p,Eigen::Matrix<double,4,1> q)
{
  double px = p(0,0);
  double py = p(1,0);
  double pz = p(2,0);
  double po = p(3,0);

  Eigen::Matrix<double,4,4> p_matrix;
  p_matrix << po, -pz,  py,  px, \
              pz,  po, -px,  py, \
             -py,  px,  po,  pz, \
             -px, -py, -pz,  po;
  return p_matrix*q;
}

Eigen::Matrix<double,4,1> quaternionMultiplicationJPL(Eigen::Matrix<double,4,1> p,Eigen::Matrix<double,4,1> q)
{
  double px = p(0,0);
  double py = p(1,0);
  double pz = p(2,0);
  double po = p(3,0);

  Eigen::Matrix<double,4,4> p_matrix;
  p_matrix << po,  pz, -py,  px, \
             -pz,  po,  px,  py, \
              py, -px,  po,  pz, \
             -px, -py, -pz,  po;
  return p_matrix*q;
}

Eigen::Matrix4d conjugate()
{
  Eigen::Vector4d conjugate_diag(-1,-1,-1,1);
  return conjugate_diag.asDiagonal();
}

Eigen::Matrix3d M(double mass)
{
  Eigen::Matrix3d Mout;
  Mout << -1.0/mass, 0, 0, \
          0, -1.0/mass, 0, \
          0, 0, 0;
  return Mout;
}

void cameraParamsToTransform(const ros::NodeHandle& nh, tf::Transform& camera_to_body)
{
  // Get camera parameters
  double qx, qy, qz, qw, cx, cy, cz;
  nh.param<double>("/camera/qx", qx, 0.5);
  nh.param<double>("/camera/qy", qy, 0.5);
  nh.param<double>("/camera/qz", qz, 0.5);
  nh.param<double>("/camera/qw", qw, 0.5);
  nh.param<double>("/camera/cx", cx, 0.0);
  nh.param<double>("/camera/cy", cy, 0.0);
  nh.param<double>("/camera/cz", cz, 0.0);

  ROS_DEBUG_STREAM("Camera Params:" <<
                  " QX = " << qx <<
                  " QY = " << qy <<
                  " QZ = " << qz <<
                  " QW = " << qw <<
                  " CX = " << cx <<
                  " CY = " << cy <<
                  " CZ = " << cz);

  // Save parameters as a tf transform
  camera_to_body.setOrigin( tf::Vector3(cx, cy, cz) );
  camera_to_body.setRotation( tf::Quaternion(qx, qy, qz, qw) );
}

} // end namespace
