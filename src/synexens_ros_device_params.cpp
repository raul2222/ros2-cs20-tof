// Associated header
//
#include "synexens_ros_driver/synexens_ros_device_params.h"

// System headers
//

// Library headers
//
#include <libsynexens3/libsynexens3.h>

// Project headers
//

sy3_error SynexensROSDeviceParams::GetDeviceConfig(sy3_config_mode_t* configuration)
{
  if (!color_enabled)
  {
    ROS_INFO_STREAM("Disabling RGB Camera");
    configuration->rgb_mode = SY3_COLOR_RESOLUTION_OFF;
  }
  else
  {
    ROS_INFO_STREAM("Setting RGB Camera Resolution: " << color_resolution);

    if (color_resolution == "480P")
    {
      configuration->rgb_mode = SY3_COLOR_RESOLUTION_640x480P;
    }
    else if (color_resolution == "1080P")
    {
      configuration->rgb_mode = SY3_COLOR_RESOLUTION_1920x1080P;
    }
    else
    {
      ROS_ERROR_STREAM("Invalid RGB Camera Resolution: " << color_resolution);
      return sy3_error::INVALID_FORMAT;
    }
  }

  if (ir_enabled || depth_enabled)
  {
    ROS_INFO_STREAM("Setting Depth Camera Resolution: " << depth_resolution);

    if (depth_resolution == "240P")
    {
      configuration->depth_mode = SY3_DEPTH_RESOLUTION_320x240P;
    }
    else if (depth_resolution == "480P")
    {
      configuration->depth_mode = SY3_DEPTH_RESOLUTION_640x480P;
    }
    else
    {
      ROS_ERROR_STREAM("Invalid Depth Camera Resolution: " << depth_resolution);
      return sy3_error::INVALID_FORMAT;
    }
  }
  else
  {
    ROS_INFO_STREAM("Disabling Depth Camera");
    configuration->depth_mode = SY3_DEPTH_RESOLUTION_OFF;
  }

  if(!depth_enabled && point_cloud_enabled)
    ROS_INFO_STREAM("Depth Camera Disabled, PointCloud Off");
  point_cloud_enabled = point_cloud_enabled && depth_enabled;
  
  configuration->enable_depth = depth_enabled;
  configuration->enable_ir = ir_enabled;
  configuration->enable_rgb = color_enabled;
  
  return sy3_error::SUCCESS;
}

void SynexensROSDeviceParams::Help()
{
#define LIST_ENTRY(param_variable, param_help_string, param_type, param_default_val)                                   \
  ROS_INFO("#param_variable - #param_type : param_help_string (#param_default_val)");

  ROS_PARAM_LIST
#undef LIST_ENTRY
}

void SynexensROSDeviceParams::Print()
{
#define LIST_ENTRY(param_variable, param_help_string, param_type, param_default_val)                                   \
  ROS_INFO_STREAM("" << #param_variable << " - " << #param_type " : " << param_variable);

  ROS_PARAM_LIST
#undef LIST_ENTRY
}
