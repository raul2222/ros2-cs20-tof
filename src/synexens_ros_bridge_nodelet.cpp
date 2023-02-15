// Associated header
//
#include "synexens_ros_driver/synexens_ros_bridge_nodelet.h"

// System headers
//

// Library headers
//
#include <pluginlib/class_list_macros.h>

// Project headers
//

PLUGINLIB_EXPORT_CLASS(Synexens_ROS_Driver::SynexensROSBridgeNodelet, nodelet::Nodelet)

namespace Synexens_ROS_Driver
{
SynexensROSBridgeNodelet::SynexensROSBridgeNodelet() : Nodelet(), sy3_device(nullptr)
{
}

SynexensROSBridgeNodelet::~SynexensROSBridgeNodelet()
{
  sy3_device.reset();
}

void SynexensROSBridgeNodelet::onInit()
{
  NODELET_INFO("Synexens ROS Nodelet Start");

  sy3_device = std::unique_ptr<SynexensROSDevice>(new SynexensROSDevice(getNodeHandle(), getPrivateNodeHandle()));

  if (sy3_device->startCameras() != sy3_error::SUCCESS)
  {
    sy3_device.reset(nullptr);
    throw nodelet::Exception("Could not start Synexens_ROS_Device!");
  }

  NODELET_INFO("Cameras started");
}
}  // namespace Azure_Kinect_ROS_Driver