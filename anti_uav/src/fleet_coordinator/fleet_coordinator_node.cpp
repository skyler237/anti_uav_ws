#include <ros/ros.h>
#include <fleet_coordinator/fleet_coordinator.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "fleet_coordinator_node");
  ros::NodeHandle nh;

  fleet_coordinator::FleetCoordinator Thing;

  ros::spin();

  return 0;
}
