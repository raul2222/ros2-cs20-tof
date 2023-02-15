// System headers
//
#include <sstream>

// Library headers
//
#include <ros/ros.h>
#include <libsynexens3/libsynexens3.h>

// Project headers
//
#include "synexens_ros_driver/synexens_ros_device.h"
using namespace sy3;

int main(int argc, char** argv)
{
  ros::init(argc, argv, "synexens_bridge");

  // Setup the synexens device
  std::shared_ptr<SynexensROSDevice> device(new SynexensROSDevice);

  sy3_error result = device->startCameras();

  if (result != sy3_error::SUCCESS)
  {
    ROS_ERROR_STREAM("Failed to start cameras");
    return -1;
  }
  
  ROS_INFO("SY3 Started");

  if (result == sy3_error::SUCCESS)
  {
    ros::spin();

    ROS_INFO("ROS Exit Started");
  }

  device.reset();

  ROS_INFO("ROS Exit");

  ros::shutdown();

  ROS_INFO("ROS Shutdown complete");

  return 0;
}