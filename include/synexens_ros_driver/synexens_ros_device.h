#ifndef LIBSYNEXENS3_ROS_DEVICE_H
#define LIBSYNEXENS3_ROS_DEVICE_H

// System headers
//
#include <atomic>
#include <mutex>
#include <thread>

// Library headers
//
#include <image_transport/image_transport.h>
#include <libsynexens3/libsynexens3.h>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Temperature.h>
#include <camera_info_manager/camera_info_manager.h>

// Project headers
//
#include "synexens_ros_driver/synexens_calibration_transform_data.h"
#include "synexens_ros_driver/synexens_ros_device_params.h"

using namespace sy3;

class SynexensROSDevice
{
 public:
  SynexensROSDevice(const ros::NodeHandle& n = ros::NodeHandle(), const ros::NodeHandle& p = ros::NodeHandle("~"));

  ~SynexensROSDevice();

  sy3_error startCameras();
  void stopCameras();

  sy3_error getDepthFrame(sy3::frameset* capture, sensor_msgs::ImagePtr& depth_frame);
  sy3_error getYuvRbgFrame(sy3::frameset* capture, sensor_msgs::ImagePtr& rgb_frame);
  sy3_error getRbgFrame(sy3::frameset* capture, sensor_msgs::ImagePtr& rgb_frame);
  sy3_error getIrFrame(sy3::frameset* capture, sensor_msgs::ImagePtr& ir_image);
  sy3_error getPointCloud(sy3::frameset* capture, sensor_msgs::PointCloud2Ptr& point_cloud);

 private:
  sy3_error renderYuvRgbToROS(sensor_msgs::ImagePtr& rgb_frame, sy3::frame* sy3_rgb_frame);
  sy3_error renderRgbToROS(sensor_msgs::ImagePtr& rgb_frame, sy3::frame* sy3_rgb_frame);
  sy3_error renderDepthToROS(sensor_msgs::ImagePtr& depth_image, sy3::frame* sy3_depth_frame);
  sy3_error renderIrToROS(sensor_msgs::ImagePtr& ir_image, sy3::frame* sy3_ir_frame);
  sy3_error fillPointCloud(sy3::depth_frame* depth_image, sensor_msgs::PointCloud2Ptr& point_cloud);

  sy3_error setOptions();
  sy3_error configStreams(const sy3_config_mode_t& configuration);
  sy3_error startStreams();

  void framePublisherThread();

  // Gets a timestap from one of the captures images
  std::chrono::microseconds getCaptureTimestamp(const sy3::frameset& capture);

  // Converts a timestamp to a ros::Time object
  ros::Time timestampToROS(const std::chrono::microseconds& timestamp_us);

  // Converts a timestamp to a ros::Time object
  ros::Time timestampToROS(const uint64_t& timestamp_us);

  // Updates the timestamp offset (stored as start_time_) between the device time and ROS time.
  void updateTimestampOffset(const std::chrono::microseconds& k4a_device_timestamp_us,
                             const std::chrono::nanoseconds& k4a_system_timestamp_ns);
  // Make an initial guess based on wall clock. The best we can do when no image timestamps are
  // available.
  void initializeTimestampOffset(const std::chrono::microseconds& k4a_device_timestamp_us);

  // ROS Node variables
  ros::NodeHandle node_;
  ros::NodeHandle private_node_;
  ros::NodeHandle node_rgb_;
  ros::NodeHandle node_ir_;

  image_transport::ImageTransport image_transport_;

  image_transport::Publisher rgb_raw_publisher_;
  ros::Publisher rgb_raw_camerainfo_publisher_;

  image_transport::Publisher rgb_rect_publisher_;

  image_transport::Publisher depth_raw_publisher_;
  ros::Publisher depth_raw_camerainfo_publisher_;

  image_transport::Publisher depth_rect_publisher_;

  image_transport::Publisher ir_raw_publisher_;
  ros::Publisher ir_raw_camerainfo_publisher_;

  ros::Publisher pointcloud_publisher_;

  // Parameters
  SynexensROSDeviceParams params_;

  std::string serial_number_;

  // SY3 device
  sy3::context* sy3_ctx_ {nullptr};
  sy3::device* sy3_device_{nullptr};
  sy3::pipeline* sy3_pline_{nullptr};
	sy3::config* sy3_cfg_{nullptr};
  sy3::process_engine* sy3_engine_{nullptr};
  SynexensCalibrationTransformData calibration_data_;

  std::chrono::nanoseconds device_to_realtime_offset_{0};

  // Thread control
  std::atomic<bool> running_;

  // Last capture timestamp for synchronizing playback capture and imu thread
  std::atomic_int64_t last_capture_time_usec_;

  // Threads
  std::thread frame_publisher_thread_;
};

#endif  // LIBSYNEXENS3_ROS_DEVICE_H
