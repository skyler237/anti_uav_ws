#include <ros/ros.h>
#include <intercept_controller/intercept_controller.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "intercept_controller_node");
  ros::NodeHandle nh;

  intercept_controller::InterceptController Thing;

  ros::spin();

  return 0;
}
