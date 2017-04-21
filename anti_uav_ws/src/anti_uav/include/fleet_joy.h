/*
 * Copyright 2015 Fadri Furrer, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Michael Burri, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Mina Kamel, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Janosch Nikolic, ASL, ETH Zurich, Switzerland
 * Copyright 2015 Markus Achtelik, ASL, ETH Zurich, Switzerland
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */


#ifndef fcu_common_joy_JOY_H_
#define fcu_common_joy_JOY_H_

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <fcu_common/Command.h>
#include <nav_msgs/Odometry.h>
#include "gazebo_msgs/ModelState.h"

struct FleetAxes {
  int roll;
  int pitch;
  int yaw;
  int x;
  int y;
  int z;
};

struct Max {
  double roll;
  double pitch;
  double roll_rate;
  double pitch_rate;
  double yaw_rate;
  double x_rate;
  double y_rate;
  double z_rate;
  double rolldot_rate;
  double pitchdot_rate;
  double yawdot_rate;
  double xdot_rate;
  double ydot_rate;
  double zdot_rate;
};

struct FleetState {
  double x;
  double y;
  double z;
  double xdot;
  double ydot;
  double zdot;

  double roll;
  double pitch;
  double yaw;
  double rolldot;
  double pitchdot;
  double yawdot;
};

struct Button{
  int index;
  bool prev_value;
};

struct Buttons {
  Button fly;
  Button mode;
  Button reset;
  Button pause;
};

enum RCMode {POSITION_MODE, VELOCITY_MODE, ACCELERATION_MODE};

class FleetJoy {
  typedef sensor_msgs::Joy::_buttons_type ButtonType;

 private:
  ros::NodeHandle nh_;
  ros::Publisher fleet_pose_pub_;
  ros::Subscriber joy_sub_;

  std::string namespace_;
  std::string command_topic_;

  std::string mav_name_;

  FleetAxes axes_;

  bool fly_mav_;

  FleetState fleet_state_;
  nav_msgs::Odometry command_msg_;
  sensor_msgs::Joy current_joy_;

  Max max_;
  Buttons buttons_;
  RCMode rc_mode_;
  geometry_msgs::Pose reset_pose_;
  geometry_msgs::Twist reset_twist_;

  double current_yaw_vel_;
  double v_yaw_step_;

  void ResetMav();
  void PauseSimulation();
  void ResumeSimulation();

  void JoyCallback(const sensor_msgs::JoyConstPtr& msg);
  void APCommandCallback(const fcu_common::CommandConstPtr& msg);
  void Publish();

 public:
  FleetJoy();
};

#endif // anti_uav_fleet_joy
