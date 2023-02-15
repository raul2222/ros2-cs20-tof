// Associated header
//
#include "synexens_ros_driver/synexens_calibration_transform_data.h"

// System headers
//
#include <stdexcept>

// Library headers
//
#include <angles/angles.h>
#include <sensor_msgs/distortion_models.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/convert.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

// Project headers
//
#include "synexens_ros_driver/synexens_ros_types.h"

void SynexensCalibrationTransformData::initialize(const SynexensROSDeviceParams& params)
{
  tf_prefix_ = params.tf_prefix;
  publishDepthToBaseTf();
}

void SynexensCalibrationTransformData::setDepthCameraCalib(const sy3_intrinsics& intrinsics)
{
  depth_camera_intrinsics_ = intrinsics;
}

void SynexensCalibrationTransformData::setColorCameraCalib(const sy3_intrinsics& intrinsics)
{
  rgb_camera_intrinsics_ = intrinsics;
}

int SynexensCalibrationTransformData::getDepthWidth()
{ 
  return depth_camera_intrinsics_.width; 
}

int SynexensCalibrationTransformData::getDepthHeight()
{ 
  return depth_camera_intrinsics_.height;
}

int SynexensCalibrationTransformData::getColorWidth()
{ 
  return rgb_camera_intrinsics_.width;
}

int SynexensCalibrationTransformData::getColorHeight()
{ 
  return rgb_camera_intrinsics_.height;
}

void SynexensCalibrationTransformData::print()
{
  ROS_INFO("SY3 Camera Intrinsics:");
  ROS_INFO("\t Depth:");
  printCameraCalibration(depth_camera_intrinsics_);

  ROS_INFO("\t Color:");
  printCameraCalibration(rgb_camera_intrinsics_);
}

void SynexensCalibrationTransformData::printCameraCalibration(sy3_intrinsics& calibration)
{
  ROS_INFO("\t\t Resolution:");
  ROS_INFO_STREAM("\t\t\t Width: " << calibration.width);
  ROS_INFO_STREAM("\t\t\t Height: " << calibration.height);

  ROS_INFO("\t\t Intrinsics:");
  ROS_INFO_STREAM("\t\t\t cx: " << calibration.ppx);
  ROS_INFO_STREAM("\t\t\t cy: " << calibration.ppy);
  ROS_INFO_STREAM("\t\t\t fx: " << calibration.fx);
  ROS_INFO_STREAM("\t\t\t fy: " << calibration.fy);
  ROS_INFO_STREAM("\t\t\t k1: " << calibration.coeffs[0]);
  ROS_INFO_STREAM("\t\t\t k2: " << calibration.coeffs[1]);
  ROS_INFO_STREAM("\t\t\t k3: " << calibration.coeffs[2]);
  ROS_INFO_STREAM("\t\t\t k4: " << calibration.coeffs[3]);
  ROS_INFO_STREAM("\t\t\t k5: " << calibration.coeffs[4]);
}

static void SetCameraInfo(const sy3_intrinsics& parameters, sensor_msgs::CameraInfo& camera_info)
{
  // The distortion parameters, size depending on the distortion model.
  // For "plumb_bob", the 5 parameters are: (k1, k2, k3, k4, k5).
  camera_info.D = {parameters.coeffs[0], parameters.coeffs[1], parameters.coeffs[2],
                   parameters.coeffs[3], parameters.coeffs[4]};

  // clang-format off
  // Intrinsic camera matrix for the raw (distorted) images.
  //     [fx  0 cx]
  // K = [ 0 fy cy]
  //     [ 0  0  1]
  // Projects 3D points in the camera coordinate frame to 2D pixel
  // coordinates using the focal lengths (fx, fy) and principal point
  // (cx, cy).
  camera_info.K = {parameters.fx,  0.0f,            parameters.ppx,
                   0.0f,           parameters.fy,   parameters.ppy,
                   0.0f,           0.0,                       1.0f};

  // Projection/camera matrix
  //     [fx'  0  cx' Tx]
  // P = [ 0  fy' cy' Ty]
  //     [ 0   0   1   0]
  // By convention, this matrix specifies the intrinsic (camera) matrix
  //  of the processed (rectified) image. That is, the left 3x3 portion
  //  is the normal camera intrinsic matrix for the rectified image.
  // It projects 3D points in the camera coordinate frame to 2D pixel
  //  coordinates using the focal lengths (fx', fy') and principal point
  //  (cx', cy') - these may differ from the values in K.
  // For monocular cameras, Tx = Ty = 0. Normally, monocular cameras will
  //  also have R = the identity and P[1:3,1:3] = K.
  camera_info.P = {parameters.fx,   0.0f,            parameters.ppx,   0.0f,
                   0.0f,            parameters.fy,   parameters.ppy,   0.0f,
                   0.0f,            0.0,                       1.0f,   0.0f};

  // Rectification matrix (stereo cameras only)
  // A rotation matrix aligning the camera coordinate system to the ideal
  // stereo image plane so that epipolar lines in both stereo images are
  // parallel.
  camera_info.R = {1.0f, 0.0f, 0.0f,
                   0.0f, 1.0f, 0.0f,
                   0.0f, 0.0f, 1.0f};
  // clang-format on
}

void SynexensCalibrationTransformData::getDepthCameraInfo(sensor_msgs::CameraInfo& camera_info, sy3_intrinsics* intrinsics)
{
  camera_info.header.frame_id = tf_prefix_ + depth_camera_frame_;
  camera_info.width = intrinsics? intrinsics->width : getColorWidth();
  camera_info.height = intrinsics? intrinsics->height : getColorHeight();
  camera_info.distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;
  SetCameraInfo(intrinsics? *intrinsics : depth_camera_intrinsics_, camera_info);
}

void SynexensCalibrationTransformData::getRgbCameraInfo(sensor_msgs::CameraInfo& camera_info, sy3_intrinsics* intrinsics)
{
  camera_info.header.frame_id = tf_prefix_ + rgb_camera_frame_;
  camera_info.width = intrinsics? intrinsics->width : getColorWidth();
  camera_info.height = intrinsics? intrinsics->height : getColorHeight();
  camera_info.distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;
  SetCameraInfo(intrinsics? *intrinsics : rgb_camera_intrinsics_, camera_info);
}

// The [0,0,0] center of the URDF, the TF frame known as "camera_base", is offset slightly from the
// [0,0,0] origin of the depth camera frame, known as "depth_camera_link" or "depth_camera_frame"
//
// Publish a TF link so the URDF model and the depth camera line up correctly
#define DEPTH_CAMERA_OFFSET_MM_X 0.0f
#define DEPTH_CAMERA_OFFSET_MM_Y 0.0f
#define DEPTH_CAMERA_OFFSET_MM_Z 1.8f  // The depth camera is shifted 1.8mm up in the depth window

tf2::Vector3 SynexensCalibrationTransformData::getDepthToBaseTranslationCorrection()
{
  // These are purely cosmetic tranformations for the URDF drawing!!
  return tf2::Vector3(DEPTH_CAMERA_OFFSET_MM_X / 1000.0f, DEPTH_CAMERA_OFFSET_MM_Y / 1000.0f,
                      DEPTH_CAMERA_OFFSET_MM_Z / 1000.0f);
}

tf2::Quaternion SynexensCalibrationTransformData::getDepthToBaseRotationCorrection()
{
  // These are purely cosmetic tranformations for the URDF drawing!!
  tf2::Quaternion ros_camera_rotation;  // ROS camera co-ordinate system requires rotating the entire camera relative to
                                        // camera_base
  tf2::Quaternion depth_rotation;       // K4A has one physical camera that is about 6 degrees downward facing.

  depth_rotation.setEuler(0, angles::from_degrees(-6.0), 0);
  ros_camera_rotation.setEuler(M_PI / -2.0f, M_PI, (M_PI / 2.0f));

  return ros_camera_rotation * depth_rotation;
}

void SynexensCalibrationTransformData::publishDepthToBaseTf()
{
  // This is a purely cosmetic transform to make the base model of the URDF look good.
  geometry_msgs::TransformStamped static_transform;

  static_transform.header.stamp = ros::Time::now();
  static_transform.header.frame_id = tf_prefix_ + camera_base_frame_;
  static_transform.child_frame_id = tf_prefix_ + depth_camera_frame_;

  tf2::Vector3 depth_translation = getDepthToBaseTranslationCorrection();
  static_transform.transform.translation.x = depth_translation.x();
  static_transform.transform.translation.y = depth_translation.y();
  static_transform.transform.translation.z = depth_translation.z();

  tf2::Quaternion depth_rotation = getDepthToBaseRotationCorrection();
  static_transform.transform.rotation.x = depth_rotation.x();
  static_transform.transform.rotation.y = depth_rotation.y();
  static_transform.transform.rotation.z = depth_rotation.z();
  static_transform.transform.rotation.w = depth_rotation.w();

  static_broadcaster_.sendTransform(static_transform);
}