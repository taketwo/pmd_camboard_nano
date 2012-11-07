#include <pmdsdk2.h>
#include <ros/ros.h>
#include <ros/console.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pmd_camboard_nano");
  ROS_INFO("Initialized camboard node.");
  return 0;
}
