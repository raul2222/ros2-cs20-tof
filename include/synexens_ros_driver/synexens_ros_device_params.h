#ifndef LIBSYNEXENS3_ROS_DEVICE_PARAMS_H
#define LIBSYNEXENS3_ROS_DEVICE_PARAMS_H

// System headers
//

// Library headers
//
#include <libsynexens3/libsynexens3.h>
#include <ros/ros.h>

// Project headers
//
#include "synexens_ros_driver/synexens_ros_types.h"

// The format of these list entries is :
//
// LIST_ENTRY(param_variable, param_help_string, param_type, param_default_val)
//
// param_variable: the variable name which will be created in the synexens_ros_device class to hold the contents of the
//    parameter
// param_help_string: a string containing help information for the parameter
// param_type: the type of the parameter
// param_default_val: the default value of the parameter
//
// Example:
// LIST_ENTRY(depth_enabled, "True if depth camera should be enabled", bool, true)
#define ROS_PARAM_LIST                                                                                                 \
  LIST_ENTRY(tf_prefix, "The prefix prepended to tf frame ID's", std::string, std::string())                           \
  LIST_ENTRY(depth_enabled, "True if depth stream should be enabled", bool, true)                                      \
  LIST_ENTRY(depth_resolution,                                                                                         \
             "The resolution of the depth frame. Options are: 240P, 480P", std::string, std::string("480P"))           \
  LIST_ENTRY(ir_enabled, "True if ir stream should be enabled", bool, true)                                            \
  LIST_ENTRY(color_enabled, "True if color stream should be enabled", bool, true)                                      \
  LIST_ENTRY(color_resolution,                                                                                         \
             "The resolution of the color frame. Options are: 480P, 1080P", std::string, std::string("1080P"))         \
  LIST_ENTRY(depth_to_rgb_enabled, "True if mapped depth in color space should be enabled", bool, false)               \
  LIST_ENTRY(rgb_to_depth_enabled, "True if mapped color in depth space should be enabled", bool, false)               \
  LIST_ENTRY(fps, "The FPS of the RGB and Depth cameras. Options are: 5, 7, 15, 30", int, 7)                           \
  LIST_ENTRY(point_cloud_enabled, "A PointCloud2 based on depth data. Requires depth_enabled=true", bool, true)        \
  LIST_ENTRY(rescale_ir_to_mono8, "True if rescale ir image to mono8 format", bool, true)                              \
  LIST_ENTRY(exposure, "The Exposure of the Depth cameras. Valid value range: > 0, Use default setting if value=-1", int, -1)     \
  LIST_ENTRY(distance_range_min, "The Min Value of Depth Map Display Distance in mm. Use default setting if value=-1", int, -1)   \
  LIST_ENTRY(distance_range_max, "The Max Value of Depth Map Display Distance in mm. Use default setting if value=-1", int, -1)   \
  LIST_ENTRY(rgb_image_flip, "0 to Disable Rgb image Flip, 1 to Enable. Use default setting if value=-1", int, -1)                \
  LIST_ENTRY(rgb_image_mirror, "0 to Disable Rgb image Mirror, 1 to Enable. Use default setting if value=-1", int, -1)            \
  LIST_ENTRY(depth_image_flip, "0 to Disable Depth image Flip, 1 to Enable. Use default setting if value=-1", int, -1)            \
  LIST_ENTRY(depth_image_mirror, "0 to Disable Depth image Mirror, 1 to Enable. Use default setting if value=-1", int, -1)        \
  LIST_ENTRY(depth_image_filter, "0 to Disable Depth image Filter, 1 to Enable. Use default setting if value=-1", int, -1)        \
  LIST_ENTRY(exposure_range_min, "The Min Value of the Depth cameras Exposure Range. Valid value range: > 0, Use default setting if value=-1", int, -1)     \
  LIST_ENTRY(exposure_range_max, "The Max Value of the Depth cameras Exposure Range. Valid value range: > 0, Use default setting if value=-1", int, -1)

using namespace sy3;

class SynexensROSDeviceParams
{
public:
  // Get a device configuration from a a set of parameters
  sy3_error GetDeviceConfig(sy3_config_mode_t* configuration);

  // Print help messages to the console
  void Help();

  // Print the value of all parameters
  void Print();

// Parameters
#define LIST_ENTRY(param_variable, param_help_string, param_type, param_default_val) param_type param_variable;
  ROS_PARAM_LIST
#undef LIST_ENTRY
};

#endif  // LIBSYNEXENS3_ROS_DEVICE_PARAMS_H
