#include "sim_analysis/sim_analysis.h"
#include "gazebo_msgs/SetModelState.h"
#include <stdio.h>

namespace sim_analysis
{

SimAnalysis::SimAnalysis() :
  nh_(ros::NodeHandle()),
  nh_private_("~")
{

  // Parameters
  net_width_ = nh_private_.param<double>("net_width", 6.0); // [m]

  world_name_ = nh_private_.param<std::string>("world_name", "fast_basic.world");

  reset_twist_.linear.x   = nh_private_.param<double>("reset_linear_twist_x", 0.0);
  reset_twist_.linear.y   = nh_private_.param<double>("reset_linear_twist_y", 0.0);
  reset_twist_.linear.z   = nh_private_.param<double>("reset_linear_twist_z", 0.0);
  reset_twist_.angular.x  = nh_private_.param<double>("reset_angular_twist_x", 0.0);
  reset_twist_.angular.y  = nh_private_.param<double>("reset_angular_twist_y", 0.0);
  reset_twist_.angular.z  = nh_private_.param<double>("reset_angular_twist_z", 0.0);

  fleet_reset_pose_.position.x = nh_private_.param<double>("fleet_reset_x", 0.0);
  fleet_reset_pose_.position.y = nh_private_.param<double>("fleet_reset_y", 0.0);
  fleet_reset_pose_.position.z = nh_private_.param<double>("fleet_reset_z", 0.0);

  fleet_reset_pose_.orientation.x = nh_private_.param<double>("fleet_reset_orient_x", 0.0);
  fleet_reset_pose_.orientation.y = nh_private_.param<double>("fleet_reset_orient_y", 0.0);
  fleet_reset_pose_.orientation.z = nh_private_.param<double>("fleet_reset_orient_z", 0.0);
  fleet_reset_pose_.orientation.w = nh_private_.param<double>("fleet_reset_orient_w", 0.0);

  intruder_name_ = nh_private_.param<std::string>("intruder_name", "shredder");
  intruder_reset_pose_.position.x = nh_private_.param<double>("intruder_reset_x", 120.0);
  intruder_reset_pose_.position.y = nh_private_.param<double>("intruder_reset_y", 0.0);
  intruder_reset_pose_.position.z = nh_private_.param<double>("intruder_reset_z", -30.0);
  intruder_reset_pose_.orientation.x = nh_private_.param<double>("intruder_reset_orient_x", 0.0);
  intruder_reset_pose_.orientation.y = nh_private_.param<double>("intruder_reset_orient_y", 0.0);
  intruder_reset_pose_.orientation.z = nh_private_.param<double>("intruder_reset_orient_z", 0.0);
  intruder_reset_pose_.orientation.w = nh_private_.param<double>("intruder_reset_orient_w", 0.0);


  uav1_name_ = nh_private_.param<std::string>("uav1_name", "shredder");
  uav1_reset_pose_.position.x = fleet_reset_pose_.position.x + net_width_/2.0;
  uav1_reset_pose_.position.y = fleet_reset_pose_.position.y - net_width_/2.0;
  uav1_reset_pose_.position.z = fleet_reset_pose_.position.z;
  uav1_reset_pose_.orientation.x = fleet_reset_pose_.orientation.x;
  uav1_reset_pose_.orientation.y = fleet_reset_pose_.orientation.y;
  uav1_reset_pose_.orientation.z = fleet_reset_pose_.orientation.z;
  uav1_reset_pose_.orientation.w = fleet_reset_pose_.orientation.w;

  uav2_name_ = nh_private_.param<std::string>("uav2_name", "shredder");
  uav2_reset_pose_.position.x = fleet_reset_pose_.position.x - net_width_/2.0;
  uav2_reset_pose_.position.y = fleet_reset_pose_.position.y - net_width_/2.0;
  uav2_reset_pose_.position.z = fleet_reset_pose_.position.z;
  uav2_reset_pose_.orientation.x = fleet_reset_pose_.orientation.x;
  uav2_reset_pose_.orientation.y = fleet_reset_pose_.orientation.y;
  uav2_reset_pose_.orientation.z = fleet_reset_pose_.orientation.z;
  uav2_reset_pose_.orientation.w = fleet_reset_pose_.orientation.w;

  uav3_name_ = nh_private_.param<std::string>("uav3_name", "shredder");
  uav3_reset_pose_.position.x = fleet_reset_pose_.position.x - net_width_/2.0;
  uav3_reset_pose_.position.y = fleet_reset_pose_.position.y + net_width_/2.0;
  uav3_reset_pose_.position.z = fleet_reset_pose_.position.z;
  uav3_reset_pose_.orientation.x = fleet_reset_pose_.orientation.x;
  uav3_reset_pose_.orientation.y = fleet_reset_pose_.orientation.y;
  uav3_reset_pose_.orientation.z = fleet_reset_pose_.orientation.z;
  uav3_reset_pose_.orientation.w = fleet_reset_pose_.orientation.w;

  uav4_name_ = nh_private_.param<std::string>("uav4_name", "shredder");
  uav4_reset_pose_.position.x = fleet_reset_pose_.position.x + net_width_/2.0;
  uav4_reset_pose_.position.y = fleet_reset_pose_.position.y + net_width_/2.0;
  uav4_reset_pose_.position.z = fleet_reset_pose_.position.z;
  uav4_reset_pose_.orientation.x = fleet_reset_pose_.orientation.x;
  uav4_reset_pose_.orientation.y = fleet_reset_pose_.orientation.y;
  uav4_reset_pose_.orientation.z = fleet_reset_pose_.orientation.z;
  uav4_reset_pose_.orientation.w = fleet_reset_pose_.orientation.w;

  last_reset_ = 0.0;

  uav1_state_.x = 0;
  uav1_state_.y = 0;
  uav1_state_.z = 0;
  uav1_state_.xdot = 0;
  uav1_state_.ydot = 0;
  uav1_state_.zdot = 0;

  uav2_state_.x = 0;
  uav2_state_.y = 0;
  uav2_state_.z = 0;
  uav2_state_.xdot = 0;
  uav2_state_.ydot = 0;
  uav2_state_.zdot = 0;

  uav3_state_.x = 0;
  uav3_state_.y = 0;
  uav3_state_.z = 0;
  uav3_state_.xdot = 0;
  uav3_state_.ydot = 0;
  uav3_state_.zdot = 0;

  uav4_state_.x = 0;
  uav4_state_.y = 0;
  uav4_state_.z = 0;
  uav4_state_.xdot = 0;
  uav4_state_.ydot = 0;
  uav4_state_.zdot = 0;



  // Set up Publishers and Subscriber
  uav1_state_sub_ = nh_.subscribe("uav1_pose", 1000, &SimAnalysis::uav1_stateCallback, this);
  uav2_state_sub_ = nh_.subscribe("uav2_pose", 1000, &SimAnalysis::uav2_stateCallback, this);
  uav3_state_sub_ = nh_.subscribe("uav3_pose", 1000, &SimAnalysis::uav3_stateCallback, this);
  uav4_state_sub_ = nh_.subscribe("uav4_pose", 1000, &SimAnalysis::uav4_stateCallback, this);

  intruder_state_sub_ = nh_.subscribe("intruder_pose", 1000, &SimAnalysis::intruder_stateCallback, this);

  // Publish results
  results_pub_ = nh_.advertise<anti_uav::InterceptResult>("results", 1000);

}

void SimAnalysis::uav1_stateCallback(const nav_msgs::OdometryConstPtr &msg) {
  uav1_state_.x = msg->pose.pose.position.x;
  uav1_state_.y = msg->pose.pose.position.y;
  uav1_state_.z = msg->pose.pose.position.z;

  uav1_state_.xdot = msg->twist.twist.linear.x;
  uav1_state_.ydot = msg->twist.twist.linear.y;
  uav1_state_.zdot = msg->twist.twist.linear.z;
}

void SimAnalysis::uav2_stateCallback(const nav_msgs::OdometryConstPtr &msg) {
  uav2_state_.x = msg->pose.pose.position.x;
  uav2_state_.y = msg->pose.pose.position.y;
  uav2_state_.z = msg->pose.pose.position.z;

  uav2_state_.xdot = msg->twist.twist.linear.x;
  uav2_state_.ydot = msg->twist.twist.linear.y;
  uav2_state_.zdot = msg->twist.twist.linear.z;
}

void SimAnalysis::uav3_stateCallback(const nav_msgs::OdometryConstPtr &msg) {
  uav3_state_.x = msg->pose.pose.position.x;
  uav3_state_.y = msg->pose.pose.position.y;
  uav3_state_.z = msg->pose.pose.position.z;

  uav3_state_.xdot = msg->twist.twist.linear.x;
  uav3_state_.ydot = msg->twist.twist.linear.y;
  uav3_state_.zdot = msg->twist.twist.linear.z;
}

void SimAnalysis::uav4_stateCallback(const nav_msgs::OdometryConstPtr &msg) {
  uav4_state_.x = msg->pose.pose.position.x;
  uav4_state_.y = msg->pose.pose.position.y;
  uav4_state_.z = msg->pose.pose.position.z;

  uav4_state_.xdot = msg->twist.twist.linear.x;
  uav4_state_.ydot = msg->twist.twist.linear.y;
  uav4_state_.zdot = msg->twist.twist.linear.z;
}

void SimAnalysis::intruder_stateCallback(const nav_msgs::OdometryConstPtr &msg)
{
    intruder_state_.x = msg->pose.pose.position.x;
    intruder_state_.y = msg->pose.pose.position.y;
    intruder_state_.z = msg->pose.pose.position.z;

    intruder_state_.xdot = msg->twist.twist.linear.x;
    intruder_state_.ydot = msg->twist.twist.linear.y;
    intruder_state_.zdot = msg->twist.twist.linear.z;

  checkIntercept();
}

void SimAnalysis::checkIntercept() {
  double now = ros::Time::now().toSec() - last_reset_;
  double dt = now - prev_time_;
  prev_time_ = now;

  static Eigen::Vector3d intruder_prev_pose(intruder_state_.x, intruder_state_.y, intruder_state_.z);
  static bool passed_fleet = false;
  static bool prev_passed_fleet = false;

  bool isIntercept = false;
  Eigen::Vector3d intercept_position;

  if(dt > 0.0)
  {
    Eigen::Vector3d uav1_pose(uav1_state_.x, uav1_state_.y, uav1_state_.z);
    Eigen::Vector3d uav2_pose(uav2_state_.x, uav2_state_.y, uav2_state_.z);
    Eigen::Vector3d uav3_pose(uav3_state_.x, uav3_state_.y, uav3_state_.z);
    Eigen::Vector3d uav4_pose(uav4_state_.x, uav4_state_.y, uav4_state_.z);
    Eigen::Vector3d intruder_pose(intruder_state_.x, intruder_state_.y, intruder_state_.z);

    Eigen::Vector3d fleet_center = (uav1_pose + uav2_pose + uav3_pose + uav4_pose)/4.0;

    // Vectors between averaged center and each vertex of the square
    Eigen::Vector3d l1 = uav1_pose - fleet_center;
    Eigen::Vector3d l2 = uav2_pose - fleet_center;
    Eigen::Vector3d l3 = uav3_pose - fleet_center;
    Eigen::Vector3d l4 = uav4_pose - fleet_center;

    // Calculate normal vectors from the vertex vectors
    Eigen::Vector3d n1 = l2.cross(l1);
    Eigen::Vector3d n2 = l3.cross(l2);
    Eigen::Vector3d n3 = l4.cross(l3);
    Eigen::Vector3d n4 = l1.cross(l4);

    // Average them to get an average normal vector for the fleet plane
    Eigen::Vector3d n_avg = (n1 + n2 + n3 + n4)/4.0;
    n_avg = n_avg.normalized();

    // Define body frame x-axis
    Eigen::Vector3d x_axis = ((uav1_pose + uav2_pose)/2.0) - fleet_center;
    x_axis = x_axis.normalized();

    // Define body frame y-axis
    Eigen::Vector3d y_axis = x_axis.cross(n_avg);

    // Project the previous intruder position into the fleet plane
    Eigen::Vector3d v_prev = intruder_prev_pose - fleet_center;
    Eigen::Vector3d v_par_prev = v_prev.dot(n_avg)*n_avg;
    Eigen::Vector3d v_perp_prev = v_prev - v_par_prev;

    double distance_prev = v_par_prev.norm(); // Perpendicular distance to fleet plane

    double proj_x_prev = v_perp_prev.dot(x_axis);
    double proj_y_prev = v_perp_prev.dot(y_axis);

    // Project the current intruder position into the fleet plane
    Eigen::Vector3d v_curr = intruder_pose - fleet_center;
    Eigen::Vector3d v_par_curr = v_curr.dot(n_avg)*n_avg;
    Eigen::Vector3d v_perp_curr = v_curr - v_par_curr;

    double distance_curr = v_par_curr.norm(); // Perpendicular distance to fleet plane

    double proj_x_curr = v_perp_curr.dot(x_axis);
    double proj_y_curr = v_perp_curr.dot(y_axis);

    Eigen::Vector2d intercept_point((proj_x_curr + proj_x_prev)/2.0, (proj_y_curr + proj_y_prev)/2.0);

    double target_dist_prev = intruder_prev_pose.norm();
    double target_dist_curr = intruder_pose.norm();
    double fleet_dist = fleet_center.norm();

    if(target_dist_prev > fleet_dist && target_dist_curr > fleet_dist) {
      passed_fleet = false;
    }
    else if (target_dist_prev <= fleet_dist && target_dist_curr <= fleet_dist) {
      passed_fleet = true;
    }

    // Publish the results when we pass the fleet
    if ((target_dist_prev > fleet_dist && fleet_dist > target_dist_curr && distance_prev > 1.0) ||
        (!prev_passed_fleet && passed_fleet)) {
      passed_fleet = true;

      // Get the current fleet x and y bounds
      double pos_x_dist = (fleet_center - (uav1_pose + uav4_pose)/2.0).norm();
      double pos_y_dist = (fleet_center - (uav2_pose + uav1_pose)/2.0).norm();
      double neg_x_dist = (fleet_center - (uav3_pose + uav2_pose)/2.0).norm();
      double neg_y_dist = (fleet_center - (uav4_pose + uav3_pose)/2.0).norm();

      // Check if the projected point is within the fleet bounds
      if(-neg_x_dist < intercept_point(0) && intercept_point(0) < pos_x_dist &&
         -neg_y_dist < intercept_point(1) && intercept_point(1) < pos_y_dist) {
        isIntercept = true;
      }
      else {
        isIntercept = false;
      }

      // Store results
      result_msg_.header.stamp = ros::Time::now();
      result_msg_.isIntercept.data = isIntercept;

      result_msg_.intercept_point.x = intercept_point(0);
      result_msg_.intercept_point.y = intercept_point(1);

      result_msg_.intercept_pose.x = fleet_center(0);
      result_msg_.intercept_pose.y = fleet_center(1);
      result_msg_.intercept_pose.z = fleet_center(2);

      result_msg_.intercept_radius = fleet_center.norm();
      result_msg_.intercept_time = now;

      // Publish the results
      publishResults();

      // TODO resetSimulation();
      resetSimulation();
    }
    else {
      passed_fleet = false;
    }


    intruder_prev_pose = intruder_pose;
    prev_passed_fleet = passed_fleet;
  }
}


void SimAnalysis::publishResults()
{
  results_pub_.publish(result_msg_);
}

void SimAnalysis::resetSimulation()
{
  // ROS_INFO("Resetting Simulation...");
  // ROS_INFO("Getting world handle for world=%s", world_name_);
  // // gazebo::physics::WorldPtr world = gazebo::physics::get_world(world_name_);
  // gazebo::physics::World world(world_name_);
  // ROS_INFO("Got world handle for world=%s", world.GetName());

	gazebo_msgs::ModelState modelstate;
	modelstate.reference_frame = (std::string) "world";
	modelstate.twist = reset_twist_;

  // Reset the intruder
	modelstate.model_name = (std::string) intruder_name_;
	modelstate.pose = intruder_reset_pose_;

  ROS_INFO("Resetting Model poses");
	ros::ServiceClient client = nh_.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
  gazebo_msgs::SetModelState setmodelstate;
	setmodelstate.request.model_state = modelstate;
	client.call(setmodelstate);


  // Reset UAV 1
  modelstate.model_name = (std::string) uav1_name_;
  modelstate.pose = uav1_reset_pose_;

  client = nh_.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
  setmodelstate.request.model_state = modelstate;
  client.call(setmodelstate);


  // Reset UAV 2
  modelstate.model_name = (std::string) uav2_name_;
  modelstate.pose = uav2_reset_pose_;

  client = nh_.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
  setmodelstate.request.model_state = modelstate;
  client.call(setmodelstate);


  // Reset UAV 3
  modelstate.model_name = (std::string) uav3_name_;
  modelstate.pose = uav3_reset_pose_;

  client = nh_.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
  setmodelstate.request.model_state = modelstate;
  client.call(setmodelstate);

  // Reset UAV 4
  modelstate.model_name = (std::string) uav4_name_;
  modelstate.pose = uav4_reset_pose_;

  client = nh_.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
  setmodelstate.request.model_state = modelstate;
  client.call(setmodelstate);

  ROS_INFO("Resetting time");
  last_reset_ = ros::Time::now().toSec();

}

} // namespace sim_analysis
