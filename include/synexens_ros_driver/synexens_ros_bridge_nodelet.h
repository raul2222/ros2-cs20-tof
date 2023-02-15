#ifndef LIBSYNEXENS3_ROS_BRIDGE_NODELET_H
#define LIBSYNEXENS3_ROS_BRIDGE_NODELET_H

// System headers
//

// Library headers
//
#include <ros/ros.h>
#include <nodelet/nodelet.h>

// Project headers
//
#include <synexens_ros_driver/synexens_ros_device.h>

namespace Synexens_ROS_Driver
{
class SynexensROSBridgeNodelet : public nodelet::Nodelet
{
public:
  SynexensROSBridgeNodelet();
  ~SynexensROSBridgeNodelet();

  virtual void onInit();

private:
  std::unique_ptr<SynexensROSDevice> sy3_device;
};
}  // namespace Synexens_ROS_Driver

#endif  // LIBSYNEXENS3_ROS_BRIDGE_NODELET_H