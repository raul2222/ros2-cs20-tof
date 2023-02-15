#ifndef LIBSYNEXENS3_CALIBRATION_TRANSFORM_DATA_H
#define LIBSYNEXENS3_CALIBRATION_TRANSFORM_DATA_H

// System headers
//
#include <vector>

// Library headers
//
#include <libsynexens3/libsynexens3.h>
#include <ros/ros.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Vector3.h>
#include <sensor_msgs/CameraInfo.h>

// Project headers
//
#include "synexens_ros_device_params.h"

using namespace sy3;

class SynexensCalibrationTransformData
{
public:
  void initialize(const SynexensROSDeviceParams& params);

  void setDepthCameraCalib(const sy3_intrinsics& intrinsics);
  void setColorCameraCalib(const sy3_intrinsics& intrinsics);
  int getDepthWidth();
  int getDepthHeight();
  int getColorWidth();
  int getColorHeight();
  void getDepthCameraInfo(sensor_msgs::CameraInfo& camera_info, sy3_intrinsics* intrinsics = nullptr);
  void getRgbCameraInfo(sensor_msgs::CameraInfo& camera_info, sy3_intrinsics* intrinsics = nullptr);
  void print();

  sy3_intrinsics rgb_camera_intrinsics_;
  sy3_intrinsics depth_camera_intrinsics_;

  std::string tf_prefix_ = "";
  std::string camera_base_frame_ = "camera_base";
  std::string rgb_camera_frame_ = "rgb_camera_link";
  std::string depth_camera_frame_ = "depth_camera_link";

private:
  void printCameraCalibration(sy3_intrinsics& calibration);
  void publishDepthToBaseTf();

  tf2::Quaternion getDepthToBaseRotationCorrection();
  tf2::Vector3 getDepthToBaseTranslationCorrection();
  
  tf2_ros::StaticTransformBroadcaster static_broadcaster_;
};

#endif // LIBSYNEXENS3_CALIBRATION_TRANSFORM_DATA_H
