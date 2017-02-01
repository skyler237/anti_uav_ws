#include <ros/ros.h>
#include <sim_analysis/sim_analysis.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "sim_analysis_node");
  ros::NodeHandle nh;

  sim_analysis::SimAnalysis Thing;

  ros::spin();

  return 0;
}
