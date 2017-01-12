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
#include <fcu_common/ExtendedCommand.h>
#include "gazebo_msgs/ModelState.h"

struct Axes {
  int roll;
  int pitch;
  int thrust;
  int yaw;
  int roll_direction;
  int pitch_direction;
  int thrust_direction;
  int yaw_direction;
};

struct Max {
  double roll;
  double pitch;
  double roll_rate;
  double pitch_rate;
  double yaw_rate;
  double aileron;
  double elevator;
  double rudder;
  double thrust;
  double altitude;
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

class Joy {
  typedef sensor_msgs::Joy::_buttons_type ButtonType;

 private:
  ros::NodeHandle nh_;
  ros::Publisher command_pub_;
  ros::Publisher extended_command_pub_;
  ros::Subscriber autopilot_command_sub_;
  ros::Subscriber joy_sub_;

  std::string namespace_;
  std::string command_topic_;
  std::string autopilot_command_topic_;

  std::string mav_name_;

  Axes axes_;

  bool fly_mav_;

  fcu_common::Command command_msg_;
  fcu_common::Command autopilot_command_;
  fcu_common::ExtendedCommand extended_command_msg_;
  sensor_msgs::Joy current_joy_;

  Max max_;
  Buttons buttons_;
  geometry_msgs::Pose reset_pose_;
  geometry_msgs::Twist reset_twist_;

  double current_yaw_vel_;
  double v_yaw_step_;
  double mass_;

  void StopMav();
  void ResetMav();
  void PauseSimulation();
  void ResumeSimulation();

  void JoyCallback(const sensor_msgs::JoyConstPtr& msg);
  void APCommandCallback(const fcu_common::CommandConstPtr& msg);
  void Publish();

 public:
  Joy();
};

#endif // fcu_common_joy_JOY_H_
