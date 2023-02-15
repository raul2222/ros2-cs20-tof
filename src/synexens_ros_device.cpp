// Associated header
//
#include "synexens_ros_driver/synexens_ros_device.h"

// System headers
//
#include <thread>

// Library headers
//
#include <libsynexens3/libsynexens3.h>
#include <angles/angles.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/distortion_models.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <unordered_map>

// Project headers
//
#include "synexens_ros_driver/synexens_ros_types.h"

using namespace ros;
using namespace sensor_msgs;
using namespace image_transport;
using namespace std;

static const std::unordered_map<sy3_color_resolution_t, std::string> color_mode_string = {
  {SY3_COLOR_RESOLUTION_640x480P, "480P"},
  {SY3_COLOR_RESOLUTION_1920x1080P, "1080P"},
};

static const std::unordered_map<sy3_depth_resolution_t, std::string> depth_mode_string = {
  {SY3_DEPTH_RESOLUTION_320x240P, "240P"},
  {SY3_DEPTH_RESOLUTION_640x480P, "480P"},
};

#define CHECK_OPTION_ERRORS(opt, e)                                      \
if(e != sy3::sy3_error::SUCCESS){                                        \
  ROS_ERROR_STREAM(#opt << " Error: " << sy3::sy3_error_to_string(e));   \
    return e;                                                            \
}                                                                        \
else{                                                                    \
  ROS_INFO_STREAM("Set Option: " << #opt);                               \
}

SynexensROSDevice::SynexensROSDevice(const NodeHandle& n, const NodeHandle& p)
  : sy3_device_(nullptr),
    node_(n),
    private_node_(p),
    node_rgb_("rgb"),
    node_ir_("ir"),
    image_transport_(n),
    last_capture_time_usec_(0)
{
  // Collect ROS parameters from the param server or from the command line
#define LIST_ENTRY(param_variable, param_help_string, param_type, param_default_val) \
  private_node_.param(#param_variable, params_.param_variable, param_default_val);
  ROS_PARAM_LIST
#undef LIST_ENTRY

  // Print all parameters
  ROS_INFO("SY3 Parameters:");
  params_.Print();

  // Setup the SY3 device
  sy3::sy3_error e;
	ROS_INFO_STREAM("libsynexens3 version: " << sy3::sy3_get_version(e));

	sy3_ctx_ = sy3::sy3_create_context(e);
	sy3::device *dev = sy3_ctx_->query_device(e);
	if (e != sy3::sy3_error::SUCCESS || !dev)
	{
    ROS_ERROR_STREAM("Failed to open a SY3 device. Cannot continue. Error: " << sy3::sy3_error_to_string(e));
    return;
	}
  sy3_device_ = dev;
  
  ROS_INFO_STREAM("SY3 Device name: " << sy3::sy3_get_device_info(dev, sy3::SY3_CAMERA_INFO_NAME, e));

  serial_number_ = sy3::sy3_get_device_info(sy3_device_, sy3::SY3_CAMERA_INFO_SERIAL_NUMBER, e);
  ROS_INFO_STREAM("SY3 Serial Number: " << serial_number_);

  ROS_INFO_STREAM("Firmware Version: " << sy3::sy3_get_device_info(dev, sy3::SY3_CAMERA_INFO_FIRMWARE_VERSION, e));

  sy3_pline_ = sy3::sy3_create_pipeline(sy3_ctx_, e);
  if (e != sy3_error::SUCCESS)
  {
    ROS_ERROR_STREAM("Failed to create pipline!" << sy3_error_to_string(e));
    return;
  }

  sy3_cfg_ = sy3::sy3_create_config(e);
  if (e != sy3_error::SUCCESS)
  {
    ROS_ERROR_STREAM("Failed to create config!" << sy3_error_to_string(e));
    return;
  }
  
  // Register our topics
  if (params_.color_enabled)
  {
    rgb_raw_publisher_ = image_transport_.advertise("rgb/image_raw", 1);
    rgb_raw_camerainfo_publisher_ = node_.advertise<CameraInfo>("rgb/camera_info", 1);
  }

  static const std::string depth_raw_topic = "depth/image_raw";
  static const std::string depth_rect_topic = "depth_to_rgb/image_raw";

  if (params_.depth_enabled)
  {
    depth_raw_publisher_ = image_transport_.advertise("depth/image_raw", 1);
    depth_raw_camerainfo_publisher_ = node_.advertise<CameraInfo>("depth/camera_info", 1);
  }
  if(params_.ir_enabled)
  {
    ir_raw_publisher_ = image_transport_.advertise("ir/image_raw", 1);
    ir_raw_camerainfo_publisher_ = node_.advertise<CameraInfo>("ir/camera_info", 1);
  }
  if (params_.point_cloud_enabled) {
    pointcloud_publisher_ = node_.advertise<PointCloud2>("points2", 1);
  }

  bool enable_mapping = params_.depth_enabled
                        && params_.color_enabled
                        && params_.depth_resolution == "480P"
                        && params_.color_resolution == "1080P";
  if(enable_mapping && params_.depth_to_rgb_enabled)
  {
    depth_rect_publisher_ = image_transport_.advertise("depth_to_rgb/image_raw", 1);
  }
  if(enable_mapping && params_.rgb_to_depth_enabled)
  {
    rgb_rect_publisher_ = image_transport_.advertise("rgb_to_depth/image_raw", 1);
  }

}

SynexensROSDevice::~SynexensROSDevice()
{
  // Start tearing down the publisher threads
  running_ = false;

  // Join the publisher thread
  ROS_INFO("Joining camera publisher thread");
  frame_publisher_thread_.join();
  ROS_INFO("Camera publisher thread joined");

  stopCameras();

  if(sy3_device_)
  {
    delete sy3_device_;
    sy3_device_ = nullptr;
  }

  if(sy3_ctx_)
  {
    delete sy3_ctx_;
    sy3_ctx_ = nullptr;
  }
}

sy3_error SynexensROSDevice::setOptions()
{
  sy3_error e = sy3_error::SUCCESS;
  const sy3::sensor* sensor = sy3_device_->get_sensor(e);
  if(e != sy3_error::SUCCESS)
  {
    ROS_ERROR_STREAM("Failed to get a SY3 sensor. Error: " << sy3::sy3_error_to_string(e));
    return e;
  }
  ROS_INFO("Set Sensor Options");
  if(params_.exposure_range_min > 0 && params_.exposure_range_max > 0 && params_.exposure_range_min < params_.exposure_range_max)
  {
    sensor->set_option(sy3::sy3_option::SY3_OPTION_EXPOSURE, (uint16_t)params_.exposure_range_max, (uint16_t)params_.exposure_range_min, e);
    CHECK_OPTION_ERRORS(SY3_OPTION_EXPOSURE, e);
  }
  if(params_.exposure > 0)
  {
    sensor->set_option(sy3::sy3_option::SY3_OPTION_EXPOSURE, (uint16_t)params_.exposure, e);
    CHECK_OPTION_ERRORS(SY3_OPTION_EXPOSURE, e);
  }
  if(params_.distance_range_min > 0 && params_.distance_range_max > 0 && params_.distance_range_min < params_.distance_range_max)
  {
    sensor->set_option(sy3::sy3_option::SY3_OPTION_DISTANCE_RANGE, (uint16_t)params_.distance_range_max, (uint16_t)params_.distance_range_min, e);
    CHECK_OPTION_ERRORS(SY3_OPTION_DISTANCE_RANGE, e);
  }
  if(params_.rgb_image_flip >= 0)
  {
    sensor->set_option(sy3::sy3_option::SY3_OPTION_RGB_IMAGE_FLIP, (uint16_t)params_.rgb_image_flip, e);
    CHECK_OPTION_ERRORS(SY3_OPTION_RGB_IMAGE_FLIP, e);
  }
  if(params_.rgb_image_mirror >= 0)
  {
    sensor->set_option(sy3::sy3_option::SY3_OPTION_RGB_IMAGE_MIRROR, (uint16_t)params_.rgb_image_mirror, e);
    CHECK_OPTION_ERRORS(SY3_OPTION_RGB_IMAGE_MIRROR, e);
  }
  if(params_.depth_image_flip >= 0)
  {
    sensor->set_option(sy3::sy3_option::SY3_OPTION_TOF_IMAGE_FLIP, (uint16_t)params_.depth_image_flip, e);
    CHECK_OPTION_ERRORS(SY3_OPTION_TOF_IMAGE_FLIP, e);
  }
  if(params_.depth_image_mirror >= 0)
  {
    sensor->set_option(sy3::sy3_option::SY3_OPTION_TOF_IMAGE_MIRROR, (uint16_t)params_.depth_image_mirror, e);
    CHECK_OPTION_ERRORS(SY3_OPTION_TOF_IMAGE_MIRROR, e);
  }
  if(params_.depth_image_filter >= 0)
  {
    sensor->set_option(sy3::sy3_option::SY3_OPTION_DEPTH_IMAGE_FILTER, (uint16_t)params_.depth_image_filter, e);
    CHECK_OPTION_ERRORS(SY3_OPTION_DEPTH_IMAGE_FILTER, e);
  }
  return sy3_error::SUCCESS;
}

sy3_error SynexensROSDevice::startCameras()
{
  sy3_error e;
  sy3_config_mode_t sy3_configuration;

  if (sy3_device_)
  {
    sy3_error result = params_.GetDeviceConfig(&sy3_configuration);
    if (result != sy3_error::SUCCESS)
    {
      ROS_ERROR("Failed to generate a device configuration. Not starting camera!");
      return result;
    }

    //Enable streams
    e = configStreams(sy3_configuration);
    if (e != sy3_error::SUCCESS)
    {
      ROS_ERROR_STREAM("Failed to config stream!" << sy3_error_to_string(e));
      return e;
    }

    // Now that we have a proposed camera configuration, we can
    // initialize the class which will take care of device calibration information
    calibration_data_.initialize(params_);
  }
  
  if (sy3_device_)
  {
    ROS_INFO_STREAM("Starting Streams");
    sy3_pline_->start(sy3_cfg_, e);
    if(e != sy3_error::SUCCESS)
    {
      ROS_ERROR_STREAM("Failed to starting Streams:" << sy3_error_to_string(e));
      return e;
    }
  }
 
  e = setOptions();
  if(e != sy3_error::SUCCESS)
  {
    ROS_ERROR_STREAM("Failed to Set Options: " << sy3_error_to_string(e));
    return e;
  }
  
  sy3_engine_ = sy3_pline_->get_process_engin(e);
  if(e != sy3_error::SUCCESS)
  {
    ROS_ERROR_STREAM("Failed to Get Process Engin: " << sy3_error_to_string(e));
    return e;
  }
  // Prevent the worker thread from exiting immediately
  running_ = true;

  // Start the thread that will poll the cameras and publish frames
  frame_publisher_thread_ = thread(&SynexensROSDevice::framePublisherThread, this);

  return sy3::sy3_error::SUCCESS;
}

void SynexensROSDevice::stopCameras()
{
  sy3_error e;
  if (sy3_device_)
  {
    // Stop the K4A SDK
    ROS_INFO("Stopping SY3 device");
    sy3_pline_->stop(e);
    ROS_INFO("SY3 device stopped");
  }
  delete sy3_pline_;
  sy3_pline_ = nullptr;
}

sy3_error SynexensROSDevice::getDepthFrame(sy3::frameset* capture, sensor_msgs::ImagePtr& depth_frame)
{
  sy3::depth_frame *sy3_depth_frame = capture->get_depth_frame();

  if (!sy3_depth_frame || !sy3_depth_frame->get_data())
  {
    ROS_INFO("Cannot render depth frame: no frame");
    return sy3_error::INCONSISTENCY_RES;
  }

  return renderDepthToROS(depth_frame, sy3_depth_frame);
}

sy3_error SynexensROSDevice::renderDepthToROS(sensor_msgs::ImagePtr& depth_image, sy3::frame* sy3_depth_frame)
{
  cv::Mat depth_frame_buffer_mat(sy3_depth_frame->get_height(), sy3_depth_frame->get_width(), CV_16UC1, sy3_depth_frame->get_data());

  depth_image = cv_bridge::CvImage(std_msgs::Header(), 
                                   sensor_msgs::image_encodings::TYPE_16UC1, 
                                   depth_frame_buffer_mat).toImageMsg();

  return sy3_error::SUCCESS;
}

sy3_error SynexensROSDevice::getIrFrame(sy3::frameset* capture, sensor_msgs::ImagePtr& ir_frame)
{
  sy3::ir_frame *sy3_ir_frame = capture->get_ir_frame();

  if (!sy3_ir_frame || !sy3_ir_frame->get_data())
  {
    ROS_INFO("Cannot render ir frame: no frame");
    return sy3_error::INCONSISTENCY_RES;
  }

  return renderIrToROS(ir_frame, sy3_ir_frame);
}

sy3_error SynexensROSDevice::renderIrToROS(sensor_msgs::ImagePtr& ir_image, sy3::frame* sy3_ir_frame)
{
  cv::Mat ir_buffer_mat(sy3_ir_frame->get_height(), sy3_ir_frame->get_width(), CV_16UC1, sy3_ir_frame->get_data());

  // Rescale the image to mono8 for visualization and usage for visual(-inertial) odometry.
  if (params_.rescale_ir_to_mono8)
  {
    cv::Mat tmp;
    cv::Mat new_image(sy3_ir_frame->get_height(), sy3_ir_frame->get_width(), CV_8UC1);
    cv::normalize(ir_buffer_mat, tmp, 0, 255, cv::NORM_MINMAX);
		cv::convertScaleAbs(tmp, new_image);
    ir_image = cv_bridge::CvImage(std_msgs::Header(), sensor_msgs::image_encodings::MONO8, new_image).toImageMsg();
  }
  else
  {
    ir_image = cv_bridge::CvImage(std_msgs::Header(), sensor_msgs::image_encodings::MONO16, ir_buffer_mat).toImageMsg();
  }

  return sy3_error::SUCCESS;
}

sy3_error SynexensROSDevice::getYuvRbgFrame(sy3::frameset* capture, sensor_msgs::ImagePtr& rgb_image)
{
  sy3::rgb_frame* sy3_rgb_frame = capture->get_rgb_frame();

  if (!sy3_rgb_frame || !sy3_rgb_frame->get_data())
  {
    ROS_INFO("Cannot render rgb frame: no frame");
    return sy3_error::INCONSISTENCY_RES;
  }

  return renderYuvRgbToROS(rgb_image, sy3_rgb_frame);
}

sy3_error SynexensROSDevice::getRbgFrame(sy3::frameset* capture, sensor_msgs::ImagePtr& rgb_image)
{
  sy3::rgb_frame* sy3_rgb_frame = capture->get_rgb_frame();

  if (!sy3_rgb_frame || !sy3_rgb_frame->get_data())
  {
    ROS_INFO("Cannot render rgb frame: no frame");
    return sy3_error::INCONSISTENCY_RES;
  }

  return renderRgbToROS(rgb_image, sy3_rgb_frame);
}

sy3_error SynexensROSDevice::renderYuvRgbToROS(sensor_msgs::ImagePtr& rgb_image, sy3::frame* sy3_rgb_frame)
{
  uint16_t* rgb_data = (uint16_t*)sy3_rgb_frame->get_data();
  int rgb_width = sy3_rgb_frame->get_width();
  int rgb_height = sy3_rgb_frame->get_height();
  cv::Mat yuvImg(rgb_height * 3 / 2, rgb_width, CV_8UC1, rgb_data);
  cv::Mat rgb_buffer_mat = cv::Mat(rgb_height, rgb_width, CV_8UC3);
  cv::cvtColor(yuvImg, rgb_buffer_mat, cv::ColorConversionCodes::COLOR_YUV2BGR_NV12);

  rgb_image = cv_bridge::CvImage(std_msgs::Header(), sensor_msgs::image_encodings::BGR8, rgb_buffer_mat).toImageMsg();
  
  return sy3_error::SUCCESS;
}

sy3_error SynexensROSDevice::renderRgbToROS(sensor_msgs::ImagePtr& rgb_image, sy3::frame* sy3_rgb_frame)
{
  cv::Mat rgb_buffer_mat(sy3_rgb_frame->get_height(), sy3_rgb_frame->get_width(), CV_8UC3, sy3_rgb_frame->get_data());

  rgb_image = cv_bridge::CvImage(std_msgs::Header(), sensor_msgs::image_encodings::BGR8, rgb_buffer_mat).toImageMsg();

  return sy3_error::SUCCESS;
}

sy3_error SynexensROSDevice::getPointCloud(sy3::frameset* capture, sensor_msgs::PointCloud2Ptr& point_cloud)
{
  sy3::depth_frame *sy3_depth_frame = capture->get_depth_frame();

  if (!sy3_depth_frame || !sy3_depth_frame->get_data())
  {
    ROS_INFO("Cannot render depth frame: no frame");
    return sy3_error::INCONSISTENCY_RES;
  }

  point_cloud->header.frame_id = calibration_data_.tf_prefix_ + calibration_data_.depth_camera_frame_;
  point_cloud->header.stamp = ros::Time::now();

  // Tranform depth image to point cloud
  return fillPointCloud(sy3_depth_frame, point_cloud);
}

sy3_error SynexensROSDevice::fillPointCloud(sy3::depth_frame* depth_image, sensor_msgs::PointCloud2Ptr& point_cloud)
{
  sy3::sy3_error e;
  if(!sy3_engine_)
  {
    ROS_INFO("Cannot get process engin");
    return sy3_error::INCONSISTENCY_RES;
  }
	sy3::points* points = sy3_engine_->comptute_points(depth_image, e);

  int length = points->get_length() / 3;
  int point_count = depth_image->get_height() * depth_image->get_width();
  //check points number
  if(point_count != length)
  {
    ROS_INFO("Point Cloud Error: invalid points number");
    return sy3_error::INCONSISTENCY_RES;
  }
  
  point_cloud->height = depth_image->get_height();
  point_cloud->width = depth_image->get_width();
  point_cloud->is_dense = false;
  point_cloud->is_bigendian = false;
  
  sensor_msgs::PointCloud2Modifier pcd_modifier(*point_cloud);
  pcd_modifier.setPointCloud2FieldsByString(1, "xyz");

  sensor_msgs::PointCloud2Iterator<float> iter_x(*point_cloud, "x");
  sensor_msgs::PointCloud2Iterator<float> iter_y(*point_cloud, "y");
  sensor_msgs::PointCloud2Iterator<float> iter_z(*point_cloud, "z");

  pcd_modifier.resize(point_count);

  point3f_t* point_cloud_buffer = (point3f_t*)points->get_points();

  for (int i = 0; i < point_count; i++, ++iter_x, ++iter_y, ++iter_z)
  {
    if (point_cloud_buffer[i].z <= 0.0f)
    {
      *iter_x = *iter_y = *iter_z = std::numeric_limits<float>::quiet_NaN();
    }
    else
    {
      constexpr float kMillimeterToMeter = 1.0 / 1000.0f;
      *iter_x = kMillimeterToMeter * point_cloud_buffer[i].x;
      *iter_y = kMillimeterToMeter * point_cloud_buffer[i].y;
      *iter_z = kMillimeterToMeter * point_cloud_buffer[i].z;
    }
  }
  delete points;

  return sy3_error::SUCCESS;
}

sy3_error SynexensROSDevice::startStreams()
{
  sy3_error e;
  sy3_pline_ = sy3::sy3_create_pipeline(sy3_ctx_, e);
  if (e != sy3_error::SUCCESS)
    return e;
  
  sy3_pline_->start(sy3_cfg_, e);
  return e;
}

sy3_error SynexensROSDevice::configStreams(const sy3_config_mode_t& configuration)
{
  sy3_error e;
  std::vector<sy3::sy3_stream> support_stream = sy3_device_->get_support_stream(e);
	for (int i = 0; i < support_stream.size(); i++)
	{
		ROS_INFO("support stream:%s ", sy3_stream_to_string(support_stream[i]));
		std::vector<sy3::sy3_format> support_format = sy3_device_->get_support_format(support_stream[i], e);
		for (int j = 0; j < support_format.size(); j++)
		{
			ROS_INFO("\t\t support format:%d x %d \n", support_format[j].width, support_format[j].height);
		}
	}

  int depth_width, depth_height;
  if(configuration.enable_depth || configuration.enable_ir)
  {
    switch (configuration.depth_mode)
    {
    case SY3_DEPTH_RESOLUTION_320x240P:
      depth_width = 320;
      depth_height = 240;
      break;
    case SY3_DEPTH_RESOLUTION_640x480P:
      depth_width = 640;
      depth_height = 480;
      break;
    default:
      depth_width = 640;
      depth_height = 480;
      break;
    }
  }
  if(configuration.enable_depth)
    sy3_cfg_->enable_stream(sy3::sy3_stream::SY3_STREAM_DEPTH, depth_width, depth_height, e);

  if(e != SUCCESS)
  {
    ROS_ERROR("Enable depth failed: %d, %d, %s", depth_width,depth_height, sy3_error_to_string(e));
    return e;
  }
    

  if(configuration.enable_ir)
    sy3_cfg_->enable_stream(sy3::sy3_stream::SY3_STREAM_IR, depth_width, depth_height, e);

  if(e != SUCCESS)
  {
    ROS_ERROR("Enable ir failed: %d, %d, %s", depth_width, depth_height, sy3_error_to_string(e));
    return e;
  }

  if(configuration.enable_rgb)
  {
    int rgb_width, rgb_height;
    switch (configuration.rgb_mode)
    {
    case SY3_COLOR_RESOLUTION_640x480P:
      rgb_width = 640;
      rgb_height = 480;
      break;
    case SY3_COLOR_RESOLUTION_1920x1080P:
      rgb_width = 1920;
      rgb_height = 1080;
      break;
    default:
      rgb_width = 640;
      rgb_height = 480;
      break;
    }
    sy3_cfg_->enable_stream(sy3::sy3_stream::SY3_STREAM_RGB, rgb_width, rgb_height, e);
    if(e != SUCCESS)
    {
      ROS_ERROR("Enable rgb failed: %d, %d, %s", rgb_width, rgb_height, sy3_error_to_string(e));
      return e;
    }
  }

  return e;
}

void SynexensROSDevice::framePublisherThread()
{
  ros::Rate loop_rate(params_.fps);
  sy3_error result;

  CameraInfo rgb_raw_camera_info;
  CameraInfo depth_raw_camera_info;
  CameraInfo ir_raw_camera_info;

  Time capture_time;
  sy3::frameset* capture { nullptr };
 
  //First frame needs longer to arrive, we wait up to 15 seconds for it
  const unsigned int firstFrameWaitTime = SY3_DEFAULT_TIMEOUT;
  //fail if we did non receive 5 consecutive frames in a row
  const unsigned int regularFrameWaitTime = 1000 * 5 / params_.fps;
  unsigned int waitTime = firstFrameWaitTime;

  while (running_ && ros::ok() && !ros::isShuttingDown())
  {
    if (sy3_pline_)
    {
      capture = sy3_pline_->wait_for_frames(waitTime, result);
      if (!capture)
      {
        //ROS_FATAL("Failed to poll cameras: node cannot continue.");
        //ros::requestShutdown();
        //return;
        ROS_INFO("Timeout to get frame");
        continue;
      }
      waitTime = regularFrameWaitTime;
      capture_time = Time::now();
    }

    ImagePtr rgb_raw_frame(new Image);
    ImagePtr depth_raw_frame(new Image);
    ImagePtr ir_raw_frame(new Image);
    ImagePtr rgb_rect_frame(new Image);
    ImagePtr depth_rect_frame(new Image);
    PointCloud2Ptr point_cloud(new PointCloud2);

    if (params_.ir_enabled)
    {
      // Only do compute if we have subscribers
      // Only create ir frame when we are using a device or we have an ir image.
      if ((ir_raw_publisher_.getNumSubscribers() > 0 || ir_raw_camerainfo_publisher_.getNumSubscribers() > 0) &&
          (capture && capture->get_ir_frame() != nullptr))
      {
        // IR images are available in all depth modes
        result = getIrFrame(capture, ir_raw_frame);

        if (result != sy3_error::SUCCESS)
        {
          ROS_ERROR("Failed to get raw IR frame");
          ros::shutdown();
          return;
        }

        // Re-sychronize the timestamps with the capture timestamp
        ir_raw_frame->header.stamp = capture_time;
        ir_raw_frame->header.frame_id = calibration_data_.tf_prefix_ + calibration_data_.depth_camera_frame_;
        ir_raw_publisher_.publish(ir_raw_frame);

        ir_raw_camera_info.header.stamp = capture_time;
        const stream_profile* profile = capture->get_ir_frame()->get_profile();
        sy3_intrinsics intrinsics = profile->get_intrinsics();
        calibration_data_.getDepthCameraInfo(ir_raw_camera_info, &intrinsics);
        ir_raw_camerainfo_publisher_.publish(ir_raw_camera_info);
      }
    }

    // Depth images
    if (params_.depth_enabled)
    {
      if ((depth_raw_publisher_.getNumSubscribers() > 0 || depth_raw_camerainfo_publisher_.getNumSubscribers() > 0) &&
          (capture && capture->get_depth_frame() != nullptr))
      {
        result = getDepthFrame(capture, depth_raw_frame);

        if (result != sy3_error::SUCCESS)
        {
          ROS_ERROR_STREAM("Failed to get raw depth frame");
          ros::shutdown();
          return;
        }

        // Re-sychronize the timestamps with the capture timestamp
        depth_raw_frame->header.stamp = capture_time;
        depth_raw_frame->header.frame_id = calibration_data_.tf_prefix_ + calibration_data_.depth_camera_frame_;
        depth_raw_publisher_.publish(depth_raw_frame);

        depth_raw_camera_info.header.stamp = capture_time;
        const stream_profile* profile = capture->get_depth_frame()->get_profile();
        sy3_intrinsics intrinsics = profile->get_intrinsics();
        calibration_data_.getDepthCameraInfo(depth_raw_camera_info, &intrinsics);
        depth_raw_camerainfo_publisher_.publish(depth_raw_camera_info);
      }
    }

    if (params_.color_enabled)
    {
      // Only create rgb frame when we are using a device or we have a color image.
      // Recordings may not have synchronized captures. For unsynchronized captures without color image skip rgb frame.
      if ((rgb_raw_publisher_.getNumSubscribers() > 0 || rgb_raw_camerainfo_publisher_.getNumSubscribers() > 0) &&
            (capture && capture->get_rgb_frame() != nullptr))
        {
          result = getYuvRbgFrame(capture, rgb_raw_frame);

          if (result != sy3_error::SUCCESS)
          {
            ROS_ERROR_STREAM("Failed to get RGB frame");
            ros::shutdown();
            return;
          }

          rgb_raw_frame->header.stamp = capture_time;
          rgb_raw_frame->header.frame_id = calibration_data_.tf_prefix_ + calibration_data_.rgb_camera_frame_;
          rgb_raw_publisher_.publish(rgb_raw_frame);

          // Re-synchronize the header timestamps since we cache the camera calibration message
          rgb_raw_camera_info.header.stamp = capture_time;
          depth_raw_camera_info.header.stamp = capture_time;
          const stream_profile* profile = capture->get_rgb_frame()->get_profile();
          sy3_intrinsics intrinsics = profile->get_intrinsics();
          calibration_data_.getDepthCameraInfo(rgb_raw_camera_info, &intrinsics);
          rgb_raw_camerainfo_publisher_.publish(rgb_raw_camera_info);
        }
    }

    // Only create pointcloud when we are using a device or we have a synchronized image.
    if(params_.point_cloud_enabled)
    {
      if (pointcloud_publisher_.getNumSubscribers() > 0 &&
        (capture && capture->get_depth_frame() != nullptr))
      {
        result = getPointCloud(capture, point_cloud);
        if (result != sy3_error::SUCCESS)
        {
          ROS_ERROR_STREAM("Failed to get Point Cloud");
          ros::shutdown();
          return;
        }
        pointcloud_publisher_.publish(point_cloud);
      }
    }
    
    bool get_depth_and_rgb = capture && capture->get_rgb_frame() != nullptr && capture->get_depth_frame() != nullptr;
    if(((rgb_rect_publisher_.getNumSubscribers() > 0) || (depth_rect_publisher_.getNumSubscribers() > 0)) && get_depth_and_rgb)
    {
      sy3::frameset* mapped_frames = sy3_engine_->align_to_rgb(capture->get_depth_frame(), capture->get_rgb_frame(), result);
      if(rgb_rect_publisher_.getNumSubscribers() > 0)
      {
        result = getRbgFrame(mapped_frames, rgb_rect_frame);

        if (result != sy3_error::SUCCESS)
        {
          ROS_ERROR_STREAM("Failed to get rect RGB frame");
          ros::shutdown();
          return;
        }

        rgb_rect_frame->header.stamp = capture_time;
        rgb_rect_frame->header.frame_id = calibration_data_.tf_prefix_ + calibration_data_.rgb_camera_frame_;
        rgb_rect_publisher_.publish(rgb_rect_frame);
      }
      if(depth_rect_publisher_.getNumSubscribers() > 0)
      {
        result = getDepthFrame(mapped_frames, depth_rect_frame);

        if (result != sy3_error::SUCCESS)
        {
          ROS_ERROR_STREAM("Failed to get rect Depth frame");
          ros::shutdown();
          return;
        }

        depth_rect_frame->header.stamp = capture_time;
        depth_rect_frame->header.frame_id = calibration_data_.tf_prefix_ + calibration_data_.rgb_camera_frame_;
        depth_rect_publisher_.publish(depth_rect_frame);
      }
      delete mapped_frames;
      mapped_frames = nullptr;
    }

    delete capture;
    capture = nullptr;

    if (loop_rate.cycleTime() > loop_rate.expectedCycleTime())
    {
      ROS_WARN_STREAM_THROTTLE(10, "Image processing thread is running behind."
                                       << std::endl
                                       << "Expected max loop time: " << loop_rate.expectedCycleTime() << std::endl
                                       << "Actual loop time: " << loop_rate.cycleTime() << std::endl);
    }

    ros::spinOnce();
    loop_rate.sleep();
  }
}
