#include "hikrobot_camera/camera_node.hpp"

#include <algorithm>
#include <chrono>
#include <cstring>
#include <functional>
#include <memory>
#include <sstream>

namespace hikrobot_camera
{

CameraNode::CameraNode(const rclcpp::NodeOptions & options)
: Node("hikrobot_camera", options)
{
  RCLCPP_INFO(
    get_logger(),
    "Starting Hikrobot MVS camera node...");

  // Declare parameters
  declare_parameter<std::string>("camera_ip", "");
  declare_parameter<std::string>("serial_number", "");
  declare_parameter<std::string>("image_topic", "/image_raw");
  declare_parameter<double>("exposure_time", 10000.0);
  declare_parameter<double>("gain", 0.0);
  declare_parameter<double>("frame_rate", 30.0);
  declare_parameter<std::string>("pixel_format", "Mono8");

  camera_ip_ = get_parameter("camera_ip").as_string();
  serial_number_ = get_parameter("serial_number").as_string();
  image_topic_ = get_parameter("image_topic").as_string();
  exposure_time_ = get_parameter("exposure_time").as_double();
  gain_ = get_parameter("gain").as_double();
  frame_rate_ = get_parameter("frame_rate").as_double();
  pixel_format_ = get_parameter("pixel_format").as_string();

  // Publisher
  image_publisher_ =
    create_publisher<sensor_msgs::msg::Image>(
    image_topic_,
    rclcpp::SensorDataQoS());

  // Parameter callback
  parameter_callback_handle_ =
    add_on_set_parameters_callback(
    std::bind(
      &CameraNode::on_parameter_change,
      this,
      std::placeholders::_1));

  if (!initialize_camera()) {
    RCLCPP_ERROR(
      get_logger(),
      "Failed to initialize Hikrobot camera.");

    reconnect_timer_ = create_wall_timer(
      std::chrono::seconds(2),
      std::bind(&CameraNode::reconnect_camera, this));

    return;
  }

  if (!connect_camera()) {
    RCLCPP_ERROR(
      get_logger(),
      "Failed to connect to Hikrobot camera.");

    reconnect_timer_ = create_wall_timer(
      std::chrono::seconds(2),
      std::bind(&CameraNode::reconnect_camera, this));

    return;
  }

  RCLCPP_INFO(
    get_logger(),
    "Hikrobot camera connected successfully.");
}


CameraNode::~CameraNode()
{
  if (reconnect_timer_) {
    reconnect_timer_->cancel();
    reconnect_timer_.reset();
  }

  disconnect_camera();
}


bool CameraNode::initialize_camera()
{
  std::memset(&device_list_, 0, sizeof(device_list_));

  int ret = MV_CC_EnumDevices(
    MV_GIGE_DEVICE | MV_USB_DEVICE,
    &device_list_);

  if (ret != MV_OK) {
    RCLCPP_ERROR(
      get_logger(),
      "MV_CC_EnumDevices failed: 0x%x",
      ret);

    return false;
  }

  RCLCPP_INFO(
    get_logger(),
    "Found %u camera(s).",
    device_list_.nDeviceNum);

  if (device_list_.nDeviceNum == 0) {
    RCLCPP_ERROR(
      get_logger(),
      "No Hikrobot camera found.");

    return false;
  }

  for (unsigned int i = 0; i < device_list_.nDeviceNum; ++i) {
    const auto * device = device_list_.pDeviceInfo[i];

    if (device == nullptr) {
      continue;
    }

    RCLCPP_INFO(
      get_logger(),
      "Camera %u: serial=%s",
      i,
      get_serial_number(device).c_str());

    if (device->nTLayerType == MV_GIGE_DEVICE) {
      const auto & info = device->SpecialInfo.stGigEInfo;

      RCLCPP_INFO(
        get_logger(),
        "Camera %u: IP=%s",
        i,
        ip_to_string(info.nCurrentIp).c_str());
    }
  }

  return true;
}


bool CameraNode::connect_camera()
{
  if (connected_) {
    return true;
  }

  if (device_list_.nDeviceNum == 0) {
    return false;
  }

  MV_CC_DEVICE_INFO * selected_device = nullptr;

  for (unsigned int i = 0; i < device_list_.nDeviceNum; ++i) {
    auto * device = device_list_.pDeviceInfo[i];

    if (device == nullptr) {
      continue;
    }

    bool serial_match = true;
    bool ip_match = true;

    if (!serial_number_.empty()) {
      serial_match =
        get_serial_number(device) == serial_number_;
    }

    if (!camera_ip_.empty()) {
      if (device->nTLayerType == MV_GIGE_DEVICE) {
        ip_match =
          ip_to_string(
            device->SpecialInfo.stGigEInfo.nCurrentIp) ==
          camera_ip_;
      } else {
        ip_match = false;
      }
    }

    if (serial_match && ip_match) {
      selected_device = device;
      break;
    }
  }

  if (selected_device == nullptr) {
    RCLCPP_ERROR(
      get_logger(),
      "No camera matches the requested IP/serial number.");

    return false;
  }

  int ret = MV_CC_CreateHandle(
    &camera_handle_,
    selected_device);

  if (ret != MV_OK) {
    RCLCPP_ERROR(
      get_logger(),
      "MV_CC_CreateHandle failed: 0x%x",
      ret);

    camera_handle_ = nullptr;
    return false;
  }

  ret = MV_CC_OpenDevice(camera_handle_);

  if (ret != MV_OK) {
    RCLCPP_ERROR(
      get_logger(),
      "MV_CC_OpenDevice failed: 0x%x",
      ret);

    MV_CC_DestroyHandle(camera_handle_);
    camera_handle_ = nullptr;

    return false;
  }

  RCLCPP_INFO(
    get_logger(),
    "Camera device opened.");

  if (!configure_camera()) {
    RCLCPP_ERROR(
      get_logger(),
      "Failed to configure camera.");

    disconnect_camera();
    return false;
  }

  ret = MV_CC_StartGrabbing(camera_handle_);

  if (ret != MV_OK) {
    RCLCPP_ERROR(
      get_logger(),
      "MV_CC_StartGrabbing failed: 0x%x",
      ret);

    disconnect_camera();
    return false;
  }

  connected_ = true;
  grabbing_ = true;

  // Create/recreate acquisition timer according to frame rate.
  if (grab_timer_) {
    grab_timer_->cancel();
    grab_timer_.reset();
  }

  const auto timer_period = std::chrono::milliseconds(
    std::max(
      static_cast<int64_t>(1),
      static_cast<int64_t>(1000.0 / frame_rate_)));

  grab_timer_ = create_wall_timer(
    timer_period,
    std::bind(&CameraNode::grab_image, this));

  if (reconnect_timer_) {
    reconnect_timer_->cancel();
    reconnect_timer_.reset();
  }

  RCLCPP_INFO(
    get_logger(),
    "Hikrobot camera connected and started grabbing.");

  return true;
}


void CameraNode::disconnect_camera()
{
  grabbing_ = false;
  connected_ = false;

  if (grab_timer_) {
    grab_timer_->cancel();
    grab_timer_.reset();
  }

  if (camera_handle_ == nullptr) {
    return;
  }

  MV_CC_StopGrabbing(camera_handle_);
  MV_CC_CloseDevice(camera_handle_);
  MV_CC_DestroyHandle(camera_handle_);

  camera_handle_ = nullptr;

  RCLCPP_INFO(
    get_logger(),
    "Camera disconnected.");
}


bool CameraNode::configure_camera()
{
  if (camera_handle_ == nullptr) {
    return false;
  }

  if (!set_float_parameter(
      "exposure_time",
      "ExposureTime",
      exposure_time_))
  {
    return false;
  }

  if (!set_float_parameter(
      "gain",
      "Gain",
      gain_))
  {
    return false;
  }

  if (!set_frame_rate(frame_rate_)) {
    return false;
  }

  if (!set_pixel_format(pixel_format_)) {
    return false;
  }

  return true;
}


void CameraNode::grab_image()
{
  if (!connected_ ||
      !grabbing_ ||
      camera_handle_ == nullptr)
  {
    return;
  }

  MV_FRAME_OUT frame;
  std::memset(&frame, 0, sizeof(frame));

  int ret = MV_CC_GetImageBuffer(
    camera_handle_,
    &frame,
    1000);

  if (ret != MV_OK) {
    RCLCPP_WARN_THROTTLE(
      get_logger(),
      *get_clock(),
      5000,
      "MV_CC_GetImageBuffer failed: 0x%x",
      ret);

    return;
  }

  publish_image(frame);

  ret = MV_CC_FreeImageBuffer(
    camera_handle_,
    &frame);

  if (ret != MV_OK) {
    RCLCPP_WARN(
      get_logger(),
      "MV_CC_FreeImageBuffer failed: 0x%x",
      ret);
  }
}


void CameraNode::publish_image(const MV_FRAME_OUT & frame)
{
  const auto & info = frame.stFrameInfo;

  if (info.enPixelType != PixelType_Gvsp_Mono8) {
    RCLCPP_WARN_THROTTLE(
      get_logger(),
      *get_clock(),
      5000,
      "Unsupported pixel format: 0x%x",
      info.enPixelType);

    return;
  }

  if (frame.pBufAddr == nullptr) {
    RCLCPP_WARN(
      get_logger(),
      "Image buffer is null.");

    return;
  }

  const size_t expected_size =
    static_cast<size_t>(info.nWidth) *
    static_cast<size_t>(info.nHeight);

  if (info.nFrameLen < expected_size) {
    RCLCPP_WARN(
      get_logger(),
      "Invalid frame length: %u, expected at least %zu.",
      info.nFrameLen,
      expected_size);

    return;
  }

  auto msg =
    std::make_unique<sensor_msgs::msg::Image>();

  msg->header.stamp = now();
  msg->header.frame_id = "hikrobot_camera";

  msg->height = info.nHeight;
  msg->width = info.nWidth;
  msg->encoding = "mono8";
  msg->is_bigendian = false;
  msg->step = info.nWidth;

  msg->data.resize(expected_size);

  std::memcpy(
    msg->data.data(),
    frame.pBufAddr,
    expected_size);

  image_publisher_->publish(std::move(msg));
}


bool CameraNode::set_float_parameter(
  const std::string & name,
  const std::string & sdk_name,
  double value)
{
  if (camera_handle_ == nullptr) {
    return false;
  }

  MVCC_FLOATVALUE float_value;
  std::memset(&float_value, 0, sizeof(float_value));

  int ret = MV_CC_GetFloatValue(
    camera_handle_,
    sdk_name.c_str(),
    &float_value);

  if (ret != MV_OK) {
    RCLCPP_ERROR(
      get_logger(),
      "Failed to query %s: 0x%x",
      sdk_name.c_str(),
      ret);

    return false;
  }

  if (value < float_value.fMin ||
      value > float_value.fMax)
  {
    RCLCPP_ERROR(
      get_logger(),
      "%s=%f is outside valid range [%f, %f].",
      name.c_str(),
      value,
      float_value.fMin,
      float_value.fMax);

    return false;
  }

  ret = MV_CC_SetFloatValue(
    camera_handle_,
    sdk_name.c_str(),
    static_cast<float>(value));

  if (ret != MV_OK) {
    RCLCPP_ERROR(
      get_logger(),
      "Failed to set %s=%f: 0x%x",
      sdk_name.c_str(),
      value,
      ret);

    return false;
  }

  return true;
}


bool CameraNode::set_frame_rate(double value)
{
  if (value <= 0.0) {
    RCLCPP_ERROR(
      get_logger(),
      "frame_rate must be greater than 0.");

    return false;
  }

  if (!set_float_parameter(
      "frame_rate",
      "AcquisitionFrameRate",
      value))
  {
    return false;
  }

  // Update ROS2 acquisition timer immediately.
  if (grab_timer_ && connected_) {
    grab_timer_->cancel();
    grab_timer_.reset();

    const auto timer_period = std::chrono::milliseconds(
      std::max(
        static_cast<int64_t>(1),
        static_cast<int64_t>(1000.0 / value)));

    grab_timer_ = create_wall_timer(
      timer_period,
      std::bind(&CameraNode::grab_image, this));
  }

  return true;
}


bool CameraNode::set_pixel_format(
  const std::string & value)
{
  if (camera_handle_ == nullptr) {
    return false;
  }

  if (value != "Mono8") {
    RCLCPP_ERROR(
      get_logger(),
      "Unsupported pixel_format: %s. Only Mono8 is supported.",
      value.c_str());

    return false;
  }

  int ret = MV_CC_SetEnumValue(
    camera_handle_,
    "PixelFormat",
    PixelType_Gvsp_Mono8);

  if (ret != MV_OK) {
    RCLCPP_ERROR(
      get_logger(),
      "Failed to set PixelFormat=Mono8: 0x%x",
      ret);

    return false;
  }

  return true;
}


void CameraNode::reconnect_camera()
{
  if (connected_) {
    return;
  }

  RCLCPP_INFO(
    get_logger(),
    "Attempting to reconnect to Hikrobot camera...");

  if (!initialize_camera()) {
    return;
  }

  connect_camera();
}


rcl_interfaces::msg::SetParametersResult
CameraNode::on_parameter_change(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";

  for (const auto & parameter : parameters) {
    const std::string & name = parameter.get_name();

    if (name == "camera_ip" ||
        name == "serial_number" ||
        name == "image_topic")
    {
      if (connected_) {
        result.successful = false;
        result.reason =
          "Camera selection and image_topic cannot be changed while running.";
        return result;
      }
    }

    if (name == "exposure_time") {
      if (!set_float_parameter(
          "exposure_time",
          "ExposureTime",
          parameter.as_double()))
      {
        result.successful = false;
        result.reason = "Failed to set exposure_time.";
        return result;
      }

      exposure_time_ = parameter.as_double();
      continue;
    }

    if (name == "gain") {
      if (!set_float_parameter(
          "gain",
          "Gain",
          parameter.as_double()))
      {
        result.successful = false;
        result.reason = "Failed to set gain.";
        return result;
      }

      gain_ = parameter.as_double();
      continue;
    }

    if (name == "frame_rate") {
      if (!set_frame_rate(parameter.as_double())) {
        result.successful = false;
        result.reason = "Failed to set frame_rate.";
        return result;
      }

      frame_rate_ = parameter.as_double();
      continue;
    }

    if (name == "pixel_format") {
      if (!set_pixel_format(parameter.as_string())) {
        result.successful = false;
        result.reason = "Failed to set pixel_format.";
        return result;
      }

      pixel_format_ = parameter.as_string();
      continue;
    }
  }

  return result;
}


std::string CameraNode::get_serial_number(
  const MV_CC_DEVICE_INFO * device) const
{
  if (device == nullptr) {
    return "";
  }

  if (device->nTLayerType == MV_GIGE_DEVICE) {
    return std::string(
      reinterpret_cast<const char *>(
        device->SpecialInfo.stGigEInfo.chSerialNumber));
  }

  if (device->nTLayerType == MV_USB_DEVICE) {
    return std::string(
      reinterpret_cast<const char *>(
        device->SpecialInfo.stUsb3VInfo.chSerialNumber));
  }

  return "";
}


std::string CameraNode::ip_to_string(
  unsigned int ip) const
{
  std::ostringstream stream;

  stream
    << ((ip >> 24) & 0xFF) << "."
    << ((ip >> 16) & 0xFF) << "."
    << ((ip >> 8) & 0xFF) << "."
    << (ip & 0xFF);

  return stream.str();
}

}  // namespace hikrobot_camera