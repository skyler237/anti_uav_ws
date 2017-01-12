/*! \file params.h
  * \author David Wheeler
  * \date June 2014
  *
  * \brief Global variables
  *
*/
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>

#ifndef relative_nav_PARAMS_H
#define relative_nav_PARAMS_H

#define PARAM_GRAVITY 9.80665
//#define PARAM_MU 1.21
#define PARAM_MASS 3.85
#define PARAM_OMEGA_SMALL_THRESHOLD 0.01

namespace relative_nav
{

inline tf::Transform loadTransform(std::string ns)
{
  ros::NodeHandle nh(ns);
  geometry_msgs::Transform msg;
  if( nh.getParam("tx", msg.translation.x) &&
      nh.getParam("ty", msg.translation.y) &&
      nh.getParam("tz", msg.translation.z) &&
      nh.getParam("qx", msg.rotation.x) &&
      nh.getParam("qy", msg.rotation.y) &&
      nh.getParam("qz", msg.rotation.z) &&
      nh.getParam("qw", msg.rotation.w))
  {
    tf::Transform out;
    tf::transformMsgToTF(msg,out);
    return out;
  }
  else
  {
    ROS_ERROR("Expected transform not found! Set /%s/tx:ty:tz:qx:qy:qz:qw",ns.c_str());
    tf::Transform out(tf::Quaternion(0,0,0,1),tf::Vector3(0,0,0));
    return out;
  }
}

inline bool loadTransform(std::string ns, tf::Transform *out)
{
  ros::NodeHandle nh(ns);
  geometry_msgs::Transform msg;
  if( nh.getParam("tx", msg.translation.x) &&
      nh.getParam("ty", msg.translation.y) &&
      nh.getParam("tz", msg.translation.z) &&
      nh.getParam("qx", msg.rotation.x) &&
      nh.getParam("qy", msg.rotation.y) &&
      nh.getParam("qz", msg.rotation.z) &&
      nh.getParam("qw", msg.rotation.w))
  {
    tf::transformMsgToTF(msg,*out);
    return true;
  }
  else
  {
    ROS_ERROR("Expected transform not found! Set /%s/tx:ty:tz:qx:qy:qz:qw",ns.c_str());
    return false;
  }
}

} // end namespace

#endif
