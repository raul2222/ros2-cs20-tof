// System headers
//
#include <sstream>

// Library headers
//
#include "rclcpp/rclcpp.hpp"
#include <libsynexens3/libsynexens3.h>

// Project headers
//
#include "synexens_ros_driver/synexens_ros_device.h"
using namespace sy3;

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("synexens_bridge");

  // Setup the synexens device
  std::shared_ptr<SynexensROSDevice> device(new SynexensROSDevice);

  sy3_error result = device->startCameras();

  if (result != sy3_error::SUCCESS)
  {
    ROS_ERROR_STREAM("Failed to start cameras");
    return -1;
  }
  
  RCLCPP_INFO(rclcpp::get_logger("SynexensRosDriver"), "SY3 Started");

  if (result == sy3_error::SUCCESS)
  {
    rclcpp::spin(node);

    RCLCPP_INFO(rclcpp::get_logger("SynexensRosDriver"), "ROS Exit Started");
  }

  device.reset();

  RCLCPP_INFO(rclcpp::get_logger("SynexensRosDriver"), "ROS Exit");

  ros::shutdown();

  RCLCPP_INFO(rclcpp::get_logger("SynexensRosDriver"), "ROS Shutdown complete");

  return 0;
}