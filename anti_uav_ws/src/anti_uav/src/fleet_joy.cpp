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

#include "fleet_joy.h"
#include "gazebo_msgs/SetModelState.h"
#include "std_srvs/Empty.h"
#include <tf/tf.h>


FleetJoy::FleetJoy() {
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  pnh.param<std::string>("command_topic", command_topic_, "fleet_pose");

  // Defaults -- should be set/overriden by calling launch file
  pnh.param<std::string>("uav1_name", mav_name_, "shredder");
  pnh.param<std::string>("uav2_name", mav_name_, "shredder");
  pnh.param<std::string>("uav3_name", mav_name_, "shredder");
  pnh.param<std::string>("uav4_name", mav_name_, "shredder");
  pnh.param<std::string>("intruder_name", mav_name_, "shredder");

  // Default to Spektrum Transmitter on Interlink
  pnh.param<int>("x_axis", axes_.x, 3);
  pnh.param<int>("y_axis", axes_.y, 0);
  pnh.param<int>("z_axis", axes_.z, 1);

  pnh.param<int>("roll_axis", axes_.roll, 2);
  pnh.param<int>("pitch_axis", axes_.pitch, 5);
  pnh.param<int>("yaw_axis", axes_.yaw, 4);

  // pnh.param<int>("x_sign", axes_.roll_direction, 1);
  // pnh.param<int>("y_sign", axes_.pitch_direction, 1);
  // pnh.param<int>("F_sign", axes_.thrust_direction, -1);
  // pnh.param<int>("z_sign", axes_.yaw_direction, 1);

  pnh.param<double>("max_roll_rate", max_.roll_rate, 0.25*M_PI/180.0);
  pnh.param<double>("max_pitch_rate", max_.pitch_rate, 0.25*M_PI/180.0);
  pnh.param<double>("max_yaw_rate", max_.yaw_rate, 0.25*M_PI/180.0);
  pnh.param<double>("max_x_rate", max_.x_rate, 0.05);
  pnh.param<double>("max_y_rate", max_.y_rate, 0.05);
  pnh.param<double>("max_z_rate", max_.z_rate, 0.05);

  pnh.param<double>("max_roll_rate", max_.rolldot_rate, 0.05*M_PI/180.0);
  pnh.param<double>("max_pitch_rate", max_.pitchdot_rate, 0.05*M_PI/180.0);
  pnh.param<double>("max_yaw_rate", max_.yawdot_rate, 0.05*M_PI/180.0);
  pnh.param<double>("max_x_rate", max_.xdot_rate, 0.01);
  pnh.param<double>("max_y_rate", max_.ydot_rate, 0.01);
  pnh.param<double>("max_z_rate", max_.zdot_rate, 0.01);

  pnh.param<double>("max_roll_angle", max_.roll, 45.0*M_PI/180.0);
  pnh.param<double>("max_pitch_angle", max_.pitch, 45.0*M_PI/180.0);

  pnh.param<double>("reset_fleet_pos_x_", reset_pose_.position.x, 0.0);
  pnh.param<double>("reset_fleet_pos_y", reset_pose_.position.y, 0.0);
  pnh.param<double>("reset_fleet_pos_z", reset_pose_.position.z, 0.0);
  pnh.param<double>("reset_orient_x", reset_pose_.orientation.x, 0.0);
  pnh.param<double>("reset_orient_x", reset_pose_.orientation.y, 0.0);
  pnh.param<double>("reset_orient_x", reset_pose_.orientation.z, 0.0);
  pnh.param<double>("reset_orient_x", reset_pose_.orientation.w, 0.0);
  pnh.param<double>("reset_linear_twist_x", reset_twist_.linear.x, 0.0);
  pnh.param<double>("reset_linear_twist_y", reset_twist_.linear.y, 0.0);
  pnh.param<double>("reset_linear_twist_z", reset_twist_.linear.z, 0.0);
  pnh.param<double>("reset_angular_twist_x", reset_twist_.angular.x, 0.0);
  pnh.param<double>("reset_angular_twist_x", reset_twist_.angular.y, 0.0);
  pnh.param<double>("reset_angular_twist_x", reset_twist_.angular.z, 0.0);

  // Sets which buttons are tied to which commands
  pnh.param<int>("button_mode_", buttons_.mode.index, 1);
  pnh.param<int>("button_reset_", buttons_.reset.index, 6);
  pnh.param<int>("button_pause_", buttons_.pause.index, 7);


  fleet_pose_pub_ = nh_.advertise<nav_msgs::Odometry>(command_topic_,10);

//  ROS_ERROR_STREAM("mass = " << mass_ <<" max_thrust = " << max_.thrust);
  rc_mode_ = POSITION_MODE;
  fleet_state_.x = 0;
  fleet_state_.y = 0;
  fleet_state_.z = 0;
  fleet_state_.xdot = 0;
  fleet_state_.ydot = 0;
  fleet_state_.zdot = 0;
  fleet_state_.roll = 0;
  fleet_state_.pitch = 0;
  fleet_state_.yaw = 0;
  fleet_state_.rolldot = 0;
  fleet_state_.pitchdot = 0;
  fleet_state_.yawdot = 0;

  command_msg_.header.frame_id = "world";
  command_msg_.child_frame_id = "fleet_state";


  namespace_ = nh_.getNamespace();
  joy_sub_ = nh_.subscribe("joy", 10, &FleetJoy::JoyCallback, this);
  buttons_.mode.prev_value=1;
  buttons_.reset.prev_value=1;

}

/* Resets the mav back to origin */
void FleetJoy::ResetMav()
{
	// ROS_INFO("Mav position reset.");
	// ros::NodeHandle n;
  //
	// gazebo_msgs::ModelState modelstate;
	// modelstate.model_name = (std::string) mav_name_;
	// modelstate.reference_frame = (std::string) "world";
	// modelstate.pose = reset_pose_;
	// modelstate.twist = reset_twist_;
  //
	// ros::ServiceClient client = n.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
	// gazebo_msgs::SetModelState setmodelstate;
	// setmodelstate.request.model_state = modelstate;
	// client.call(setmodelstate);

  ROS_INFO("Fleet state reset.");
  fleet_state_.x = 0;
  fleet_state_.y = 0;
  fleet_state_.z = 0;
  fleet_state_.xdot = 0;
  fleet_state_.ydot = 0;
  fleet_state_.zdot = 0;
  fleet_state_.roll = 0;
  fleet_state_.pitch = 0;
  fleet_state_.yaw = 0;
  fleet_state_.rolldot = 0;
  fleet_state_.pitchdot = 0;
  fleet_state_.yawdot = 0;
}

// Pauses the gazebo physics and time
void FleetJoy::PauseSimulation()
{
	ROS_INFO("Simulation paused.");
	ros::NodeHandle n;

	ros::ServiceClient client = n.serviceClient<std_srvs::Empty>("/gazebo/pause_physics");
	std_srvs::Empty pauseSim;
	client.call(pauseSim);
}

// Resumes the gazebo physics and time
void FleetJoy::ResumeSimulation()
{
	ROS_INFO("Simulation resumed.");
	ros::NodeHandle n;

	ros::ServiceClient client = n.serviceClient<std_srvs::Empty>("/gazebo/unpause_physics");
	std_srvs::Empty resumeSim;
	client.call(resumeSim);
}

void FleetJoy::JoyCallback(const sensor_msgs::JoyConstPtr& msg) {
  static int mode = -1;
  static bool paused = true;
//  if(fly_mav_)
//  {
    current_joy_ = *msg;

    // Adjust state based on joy input
    switch(rc_mode_) {
      case POSITION_MODE:
        fleet_state_.x += msg->axes[axes_.x]*max_.x_rate;
        fleet_state_.y += msg->axes[axes_.y]*max_.y_rate;
        fleet_state_.z += -1.0*msg->axes[axes_.z]*max_.z_rate;
        fleet_state_.roll += msg->axes[axes_.roll]*max_.roll_rate;
        fleet_state_.pitch += msg->axes[axes_.pitch]*max_.pitch_rate;
        fleet_state_.yaw += msg->axes[axes_.yaw]*max_.yaw_rate;
      break;

      case VELOCITY_MODE:
        fleet_state_.xdot += msg->axes[axes_.x]*max_.xdot_rate;
        fleet_state_.ydot += msg->axes[axes_.y]*max_.ydot_rate;
        fleet_state_.zdot += msg->axes[axes_.z]*max_.zdot_rate;
        fleet_state_.rolldot += msg->axes[axes_.roll]*max_.rolldot_rate;
        fleet_state_.pitchdot += msg->axes[axes_.pitch]*max_.pitchdot_rate;
        fleet_state_.yawdot += msg->axes[axes_.yaw]*max_.yawdot_rate;

        fleet_state_.x += fleet_state_.xdot*max_.x_rate;
        fleet_state_.y += fleet_state_.ydot*max_.y_rate;
        fleet_state_.z += -1.0*fleet_state_.zdot*max_.z_rate;
        fleet_state_.roll += fleet_state_.rolldot*max_.roll_rate;
        fleet_state_.pitch += fleet_state_.pitchdot*max_.pitch_rate;
        fleet_state_.yaw += fleet_state_.yawdot*max_.yaw_rate;
      break;

      // case ACCELERATION_MODE:

      break;

      default:
      ROS_INFO("WARNING: unexpected RC state.");

    }


    // ROS_INFO("fleet_state:\n");
    // ROS_INFO("x=%f, y=%f, z=%f,   roll=%f, pitch=%f, yaw=%f",
    //           fleet_state_.x, fleet_state_.y, fleet_state_.z, fleet_state_.roll, fleet_state_.pitch, fleet_state_.yaw);

    // Resets the mav to the origin when start button (button 9) is pressed (if using xbox controller)
    if(msg->buttons[buttons_.reset.index]==0 && buttons_.reset.prev_value==1) // button release
    {
    	ResetMav();
    }
    buttons_.reset.prev_value = msg->buttons[buttons_.reset.index];

    // Pauses/Unpauses the simulation
    if(msg->buttons[buttons_.pause.index]==0 && buttons_.pause.prev_value==1) // button release
    {
    	if(!paused)
    	{
    		PauseSimulation();
    		paused = true;
    	}
    	else
    	{
    		ResumeSimulation();
    		paused = false;
    	}

    }
    buttons_.pause.prev_value = msg->buttons[buttons_.pause.index];

    if(msg->buttons[buttons_.mode.index]==0 && buttons_.mode.prev_value==1){ // button release
      mode = (mode+1)%2;
      if(mode == 0)
      {
        ROS_INFO("Position Mode");
        rc_mode_ = POSITION_MODE;
      }
      else if(mode == 1)
      {
        ROS_INFO("Velocity Mode");
        rc_mode_ = VELOCITY_MODE;
      }
      // else if (mode == 2)
      // {
      //   ROS_INFO("Acceleration Mode");
      //   rc_mode_ = ACCELERATION_MODE;
      // }
    }
    buttons_.mode.prev_value = msg->buttons[buttons_.mode.index];

  Publish();
}

void FleetJoy::Publish() {
  command_msg_.header.stamp = ros::Time::now();

  command_msg_.pose.pose.position.x = fleet_state_.x;
  command_msg_.pose.pose.position.y = fleet_state_.y;
  command_msg_.pose.pose.position.z = fleet_state_.z;

  geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromRollPitchYaw(fleet_state_.roll, fleet_state_.pitch, fleet_state_.yaw);
  command_msg_.pose.pose.orientation = odom_quat;

  fleet_pose_pub_.publish(command_msg_);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "anti_uav");
  FleetJoy FleetJoy;

  ros::spin();

  return 0;
}
