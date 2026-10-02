#include "hikrobot_camera/camera_node.hpp"

#include <algorithm>
#include <chrono>
#include <cstring>
#include <functional>
#include <memory>
#include <sstream>

namespace hikrobot_camera
{

CameraNode::CameraNode(
  const rclcpp::NodeOptions & options)
: Node("hikrobot_camera", options)
{
  RCLCPP_INFO(
    get_logger(),
    "Starting Hikrobot MVS camera node...");

  // ============================================================
  // Declare parameters
  // ============================================================

  declare_parameter<std::string>(
    "camera_ip", "");

  declare_parameter<std::string>(
    "serial_number", "");

  declare_parameter<std::string>(
    "image_topic", "/image_raw");

  declare_parameter<double>(
    "exposure_time", 10000.0);

  declare_parameter<double>(
    "gain", 0.0);

  declare_parameter<double>(
    "frame_rate", 30.0);

  declare_parameter<std::string>(
    "pixel_format", "Mono8");

  // ============================================================
  // Read parameters
  // ============================================================

  camera_ip_ =
    get_parameter("camera_ip").as_string();

  serial_number_ =
    get_parameter("serial_number").as_string();

  image_topic_ =
    get_parameter("image_topic").as_string();

  exposure_time_ =
    get_parameter("exposure_time").as_double();

  gain_ =
    get_parameter("gain").as_double();

  frame_rate_ =
    get_parameter("frame_rate").as_double();

  pixel_format_ =
    get_parameter("pixel_format").as_string();

  // ============================================================
  // Publisher
  // ============================================================

  image_publisher_ =
    create_publisher<sensor_msgs::msg::Image>(
      image_topic_,
      rclcpp::SensorDataQoS());

  // ============================================================
  // Parameter callback
  // ============================================================

  parameter_callback_handle_ =
    add_on_set_parameters_callback(
      std::bind(
        &CameraNode::on_parameter_change,
        this,
        std::placeholders::_1));

  // ============================================================
  // Camera discovery
  // ============================================================

  if (!initialize_camera()) {
    RCLCPP_WARN(
      get_logger(),
      "No Hikrobot camera is currently available. "
      "Check the USB connection and close the MVS "
      "client if it is running.");
  }

  // ============================================================
  // Initial connection
  // ============================================================

  if (device_list_.nDeviceNum > 0) {

    if (!connect_camera()) {
      RCLCPP_ERROR(
        get_logger(),
        "Failed to connect to Hikrobot camera.");
    }
  }

  // ============================================================
  // Connection monitor
  // ============================================================

  connection_monitor_timer_ =
    create_wall_timer(
      std::chrono::seconds(1),
      std::bind(
        &CameraNode::monitor_camera_connection,
        this));

  // ============================================================
  // Reconnect timer
  // ============================================================

  if (!connected_) {
    start_reconnect_timer();
  }

  if (connected_) {
    RCLCPP_INFO(
      get_logger(),
      "Hikrobot camera connected successfully.");
  }
}


// ================================================================
// Destructor
// ================================================================

CameraNode::~CameraNode()
{
  if (connection_monitor_timer_) {
    connection_monitor_timer_->cancel();
    connection_monitor_timer_.reset();
  }

  if (reconnect_timer_) {
    reconnect_timer_->cancel();
    reconnect_timer_.reset();
  }

  disconnect_camera();
}


// ================================================================
// Enumerate cameras
// ================================================================

bool CameraNode::initialize_camera()
{
  std::memset(
    &device_list_,
    0,
    sizeof(device_list_));

  int ret =
    MV_CC_EnumDevices(
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
    return false;
  }

  for (unsigned int i = 0;
       i < device_list_.nDeviceNum;
       ++i) {

    const auto * device =
      device_list_.pDeviceInfo[i];

    if (device == nullptr) {
      continue;
    }

    RCLCPP_INFO(
      get_logger(),
      "Camera %u: serial=%s",
      i,
      get_serial_number(device).c_str());

    if (device->nTLayerType ==
        MV_GIGE_DEVICE) {

      RCLCPP_INFO(
        get_logger(),
        "Camera %u: IP=%s",
        i,
        ip_to_string(
          device->SpecialInfo
            .stGigEInfo
            .nCurrentIp).c_str());
    }
  }

  return true;
}


// ================================================================
// Connect camera
// ================================================================

bool CameraNode::connect_camera()
{
  if (connected_) {
    return true;
  }

  if (device_list_.nDeviceNum == 0) {
    return false;
  }

  std::vector<MV_CC_DEVICE_INFO *> matches;

  // --------------------------------------------------------------
  // Camera selection
  // --------------------------------------------------------------

  for (unsigned int i = 0;
       i < device_list_.nDeviceNum;
       ++i) {

    auto * device =
      device_list_.pDeviceInfo[i];

    if (device_matches(device)) {
      matches.push_back(device);
    }
  }

  if (matches.empty()) {
    if (serial_number_.empty() && camera_ip_.empty()) {
      RCLCPP_ERROR(
        get_logger(),
        "No camera identifier is configured; "
        "set camera_ip or serial_number.");
    } else {
      RCLCPP_ERROR(
        get_logger(),
        "No camera matches the configured "
        "IP/serial number.");
    }

    std::ostringstream discovered_devices;

    for (unsigned int i = 0;
         i < device_list_.nDeviceNum;
         ++i) {

      if (i > 0) {
        discovered_devices << ", ";
      }

      const auto * device = device_list_.pDeviceInfo[i];

      if (device == nullptr) {
        discovered_devices << "unknown";
        continue;
      }

      discovered_devices
        << "serial="
        << get_serial_number(device);

      if (device->nTLayerType == MV_GIGE_DEVICE) {
        discovered_devices
          << " ip="
          << ip_to_string(
               device->SpecialInfo
                 .stGigEInfo
                 .nCurrentIp);
      }
    }

    RCLCPP_ERROR(
      get_logger(),
      "Discovered %u device(s): %s.",
      device_list_.nDeviceNum,
      discovered_devices.str().c_str());

    return false;
  }

  if (matches.size() > 1) {
    RCLCPP_ERROR(
      get_logger(),
      "Multiple cameras (%zu) match the configured "
      "identifier; set camera_ip and serial_number "
      "to select one device.",
      matches.size());
    return false;
  }

  MV_CC_DEVICE_INFO * selected_device =
    matches.front();

  // --------------------------------------------------------------
  // Create handle
  // --------------------------------------------------------------

  int ret =
    MV_CC_CreateHandle(
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

  // --------------------------------------------------------------
  // Open device
  // --------------------------------------------------------------

  ret =
    MV_CC_OpenDevice(
      camera_handle_);

  if (ret != MV_OK) {

    if (ret ==
          static_cast<int>(MV_E_DEV_BUSY) ||
        ret ==
          static_cast<int>(MV_E_BUSY)) {
      RCLCPP_ERROR(
        get_logger(),
        "MV_CC_OpenDevice failed: 0x%x. "
        "The camera is busy or occupied by "
        "another application.",
        ret);
    } else if (
      ret ==
      static_cast<int>(MV_E_ACCESS_DENIED)) {
      RCLCPP_ERROR(
        get_logger(),
        "MV_CC_OpenDevice failed: 0x%x. "
        "Permission denied; check MVS udev rules "
        "or user permissions.",
        ret);
    } else {
      RCLCPP_ERROR(
        get_logger(),
        "MV_CC_OpenDevice failed: 0x%x",
        ret);
    }

    MV_CC_DestroyHandle(
      camera_handle_);

    camera_handle_ = nullptr;

    return false;
  }

  RCLCPP_INFO(
    get_logger(),
    "Camera device opened.");

  // --------------------------------------------------------------
  // Configure
  // --------------------------------------------------------------

  if (!configure_camera()) {

    RCLCPP_ERROR(
      get_logger(),
      "Failed to configure camera.");

    disconnect_camera();

    return false;
  }

  // --------------------------------------------------------------
  // Start acquisition
  // --------------------------------------------------------------

  ret =
    MV_CC_StartGrabbing(
      camera_handle_);

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

  consecutive_grab_failures_ = 0;

  // --------------------------------------------------------------
  // Acquisition timer
  // --------------------------------------------------------------

  if (grab_timer_) {
    grab_timer_->cancel();
    grab_timer_.reset();
  }

  const auto timer_period =
    std::chrono::milliseconds(
      std::max(
        static_cast<int64_t>(1),
        static_cast<int64_t>(
          1000.0 / frame_rate_)));

  grab_timer_ =
    create_wall_timer(
      timer_period,
      std::bind(
        &CameraNode::grab_image,
        this));

  // --------------------------------------------------------------
  // Reconnect timer no longer needed
  // --------------------------------------------------------------

  if (reconnect_timer_) {
    reconnect_timer_->cancel();
    reconnect_timer_.reset();
  }

  RCLCPP_INFO(
    get_logger(),
    "Hikrobot camera connected and started grabbing.");

  return true;
}


// ================================================================
// Disconnect
// ================================================================

void CameraNode::disconnect_camera()
{
  grabbing_ = false;
  connected_ = false;

  consecutive_grab_failures_ = 0;

  if (grab_timer_) {
    grab_timer_->cancel();
    grab_timer_.reset();
  }

  if (camera_handle_ == nullptr) {
    return;
  }

  RCLCPP_INFO(
    get_logger(),
    "Disconnecting Hikrobot camera...");

  MV_CC_StopGrabbing(
    camera_handle_);

  MV_CC_CloseDevice(
    camera_handle_);

  MV_CC_DestroyHandle(
    camera_handle_);

  camera_handle_ = nullptr;

  RCLCPP_INFO(
    get_logger(),
    "Camera disconnected.");
}


// ================================================================
// Camera configuration
// ================================================================

bool CameraNode::configure_camera()
{
  if (camera_handle_ == nullptr) {
    return false;
  }

  // Manual exposure/gain require automatic modes off.
  if (!set_enum_parameter(
      "ExposureAuto", 0, true)) {
    return false;
  }

  if (!set_enum_parameter(
      "GainAuto", 0, true)) {
    return false;
  }

  if (!set_float_parameter(
      "exposure_time",
      "ExposureTime",
      exposure_time_)) {
    return false;
  }

  if (!set_float_parameter(
      "gain",
      "Gain",
      gain_)) {
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


// ================================================================
// Grab image
// ================================================================

void CameraNode::grab_image()
{
  if (!connected_ ||
      !grabbing_ ||
      camera_handle_ == nullptr) {
    return;
  }

  MV_FRAME_OUT frame;

  std::memset(
    &frame,
    0,
    sizeof(frame));

  // Short timeout prevents the acquisition callback
  // from remaining blocked for a long period.
  int ret =
    MV_CC_GetImageBuffer(
      camera_handle_,
      &frame,
      100);

  if (ret != MV_OK) {

    ++consecutive_grab_failures_;

    if (consecutive_grab_failures_ >= 10) {
      RCLCPP_ERROR(
        get_logger(),
        "Too many consecutive grab failures; "
        "reconnecting the camera.");

      disconnect_camera();
      start_reconnect_timer();
      return;
    }

    RCLCPP_WARN_THROTTLE(
      get_logger(),
      *get_clock(),
      5000,
      "MV_CC_GetImageBuffer failed: 0x%x "
      "(consecutive failures: %d)",
      ret,
      consecutive_grab_failures_);

    return;
  }

  consecutive_grab_failures_ = 0;

  publish_image(frame);

  ret =
    MV_CC_FreeImageBuffer(
      camera_handle_,
      &frame);

  if (ret != MV_OK) {
    RCLCPP_WARN(
      get_logger(),
      "MV_CC_FreeImageBuffer failed: 0x%x",
      ret);
  }
}


// ================================================================
// Monitor camera connection
// ================================================================

void CameraNode::monitor_camera_connection()
{
  if (reconnecting_) {
    return;
  }

  if (connected_) {

    if (!current_camera_present()) {

      RCLCPP_ERROR(
        get_logger(),
        "Hikrobot camera is no longer detected.");

      disconnect_camera();
      start_reconnect_timer();
    }

    return;
  }

  start_reconnect_timer();
}


// ================================================================
// Check whether the selected camera exists
// ================================================================

bool CameraNode::current_camera_present() const
{
  MV_CC_DEVICE_INFO_LIST current_list;

  std::memset(
    &current_list,
    0,
    sizeof(current_list));

  int ret =
    MV_CC_EnumDevices(
      MV_GIGE_DEVICE | MV_USB_DEVICE,
      &current_list);

  if (ret != MV_OK) {
    return false;
  }

  for (unsigned int i = 0;
       i < current_list.nDeviceNum;
       ++i) {

    if (device_matches(
          current_list.pDeviceInfo[i])) {
      return true;
    }
  }

  return false;
}


// ================================================================
// Reconnect camera
// ================================================================

void CameraNode::reconnect_camera()
{
  if (connected_ || reconnecting_) {
    return;
  }

  reconnecting_ = true;

  RCLCPP_INFO(
    get_logger(),
    "Attempting to reconnect to Hikrobot camera...");

  bool initialized =
    initialize_camera();

  if (initialized &&
      device_list_.nDeviceNum > 0) {

    if (connect_camera()) {

      RCLCPP_INFO(
        get_logger(),
        "Hikrobot camera reconnected successfully.");

      reconnecting_ = false;

      return;
    }
  }

  RCLCPP_WARN(
    get_logger(),
    "Reconnect attempt failed. "
    "Will retry later.");

  reconnecting_ = false;
}


// ================================================================
// Publish ROS Image
// ================================================================

void CameraNode::publish_image(
  const MV_FRAME_OUT & frame)
{
  const auto & info =
    frame.stFrameInfo;

  if (info.enPixelType !=
      PixelType_Gvsp_Mono8) {

    RCLCPP_WARN_THROTTLE(
      get_logger(),
      *get_clock(),
      5000,
      "Unsupported pixel format: 0x%lx",
      static_cast<unsigned long>(
        info.enPixelType));

    return;
  }

  if (frame.pBufAddr == nullptr) {

    RCLCPP_WARN(
      get_logger(),
      "Image buffer is null.");

    return;
  }

  const size_t expected_size =
    static_cast<size_t>(
      info.nWidth) *
    static_cast<size_t>(
      info.nHeight);

  if (info.nFrameLen < expected_size) {

    RCLCPP_WARN(
      get_logger(),
      "Invalid frame length: %u, "
      "expected at least %zu.",
      info.nFrameLen,
      expected_size);

    return;
  }

  auto msg =
    std::make_unique<
      sensor_msgs::msg::Image>();

  msg->header.stamp = now();

  msg->header.frame_id =
    "hikrobot_camera";

  msg->height =
    info.nHeight;

  msg->width =
    info.nWidth;

  msg->encoding =
    "mono8";

  msg->is_bigendian =
    false;

  msg->step =
    info.nWidth;

  msg->data.resize(
    expected_size);

  std::memcpy(
    msg->data.data(),
    frame.pBufAddr,
    expected_size);

  image_publisher_->publish(
    std::move(msg));
}


// ================================================================
// Set floating-point parameter
// ================================================================

bool CameraNode::set_float_parameter(
  const std::string & name,
  const std::string & sdk_name,
  double value)
{
  if (camera_handle_ == nullptr) {
    return false;
  }

  MVCC_FLOATVALUE float_value;

  std::memset(
    &float_value,
    0,
    sizeof(float_value));

  int ret =
    MV_CC_GetFloatValue(
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
      value > float_value.fMax) {

    RCLCPP_ERROR(
      get_logger(),
      "%s=%f is outside valid range "
      "[%f, %f].",
      name.c_str(),
      value,
      float_value.fMin,
      float_value.fMax);

    return false;
  }

  ret =
    MV_CC_SetFloatValue(
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


// ================================================================
// Set enum parameter
// ================================================================

bool CameraNode::set_enum_parameter(
  const std::string & sdk_name,
  unsigned int value,
  bool allow_unsupported)
{
  if (camera_handle_ == nullptr) {
    RCLCPP_ERROR(
      get_logger(),
      "Cannot set %s: camera is not connected.",
      sdk_name.c_str());

    return false;
  }

  int ret =
    MV_CC_SetEnumValue(
      camera_handle_,
      sdk_name.c_str(),
      value);

  if (ret != MV_OK) {
    if (allow_unsupported &&
        ret ==
          static_cast<int>(MV_E_NOT_IMPLEMENTED)) {
      RCLCPP_WARN(
        get_logger(),
        "%s is not supported by this camera; "
        "continuing without changing it.",
        sdk_name.c_str());

      return true;
    }

    RCLCPP_ERROR(
      get_logger(),
      "Failed to set %s=%u: 0x%x",
      sdk_name.c_str(),
      value,
      ret);

    return false;
  }

  return true;
}


// ================================================================
// Set frame rate
// ================================================================

bool CameraNode::set_frame_rate(
  double value)
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
      value)) {
    return false;
  }

  if (grab_timer_ && connected_) {

    grab_timer_->cancel();
    grab_timer_.reset();

    const auto timer_period =
      std::chrono::milliseconds(
        std::max(
          static_cast<int64_t>(1),
          static_cast<int64_t>(
            1000.0 / value)));

    grab_timer_ =
      create_wall_timer(
        timer_period,
        std::bind(
          &CameraNode::grab_image,
          this));
  }

  return true;
}


// ================================================================
// Set pixel format
// ================================================================

bool CameraNode::set_pixel_format(
  const std::string & value)
{
  if (camera_handle_ == nullptr) {
    return false;
  }

  if (value != "Mono8") {

    RCLCPP_ERROR(
      get_logger(),
      "Unsupported pixel_format: %s. "
      "Only Mono8 is currently supported.",
      value.c_str());

    return false;
  }

  return set_enum_parameter(
    "PixelFormat",
    PixelType_Gvsp_Mono8);
}


// ================================================================
// Dynamic parameter callback
// ================================================================

rcl_interfaces::msg::SetParametersResult
CameraNode::on_parameter_change(
  const std::vector<rclcpp::Parameter> & parameters)
{
  rcl_interfaces::msg::SetParametersResult result;

  result.successful = true;
  result.reason = "success";

  for (const auto & parameter :
       parameters) {

    const std::string & name =
      parameter.get_name();

    // Camera selection and topic are startup-only
    // parameters.
    if (name == "camera_ip" ||
        name == "serial_number" ||
        name == "image_topic") {

      result.successful = false;
      result.reason =
        name +
        " cannot be changed at runtime; "
        "restart the node with the new value.";

      return result;
    }

    if (camera_handle_ == nullptr) {
      result.successful = false;
      result.reason =
        name +
        " cannot be changed while no camera "
        "is connected.";

      return result;
    }

    if (name == "exposure_time") {

      const double value =
        parameter.as_double();

      if (!set_enum_parameter(
          "ExposureAuto", 0, true)) {
        result.successful = false;
        result.reason =
          "Failed to disable automatic exposure "
          "before setting exposure_time.";

        return result;
      }

      if (!set_float_parameter(
          "exposure_time",
          "ExposureTime",
          value)) {

        result.successful = false;
        result.reason =
          "Failed to set exposure_time.";

        return result;
      }

      exposure_time_ = value;

      continue;
    }

    if (name == "gain") {

      const double value =
        parameter.as_double();

      if (!set_enum_parameter(
          "GainAuto", 0, true)) {
        result.successful = false;
        result.reason =
          "Failed to disable automatic gain "
          "before setting gain.";

        return result;
      }

      if (!set_float_parameter(
          "gain",
          "Gain",
          value)) {

        result.successful = false;
        result.reason =
          "Failed to set gain.";

        return result;
      }

      gain_ = value;

      continue;
    }

    if (name == "frame_rate") {

      const double value =
        parameter.as_double();

      if (!set_frame_rate(value)) {

        result.successful = false;
        result.reason =
          "Failed to set frame_rate.";

        return result;
      }

      frame_rate_ = value;

      continue;
    }

    if (name == "pixel_format") {

      const std::string value =
        parameter.as_string();

      if (!set_pixel_format(value)) {

        result.successful = false;
        result.reason =
          "Failed to set pixel_format.";

        return result;
      }

      pixel_format_ = value;

      continue;
    }
  }

  return result;
}


// ================================================================
// Device matching
// ================================================================

bool CameraNode::device_matches(
  const MV_CC_DEVICE_INFO * device) const
{
  if (device == nullptr) {
    return false;
  }

  if (!serial_number_.empty() &&
      get_serial_number(device) != serial_number_) {
    return false;
  }

  if (!camera_ip_.empty()) {

    if (device->nTLayerType != MV_GIGE_DEVICE) {
      return false;
    }

    if (ip_to_string(
          device->SpecialInfo
            .stGigEInfo
            .nCurrentIp) != camera_ip_) {
      return false;
    }
  }

  return true;
}


// ================================================================
// Start reconnect timer
// ================================================================

void CameraNode::start_reconnect_timer()
{
  if (!reconnect_timer_) {
    reconnect_timer_ =
      create_wall_timer(
        std::chrono::seconds(2),
        std::bind(
          &CameraNode::reconnect_camera,
          this));
  }
}


// ================================================================
// Get serial number
// ================================================================

std::string CameraNode::get_serial_number(
  const MV_CC_DEVICE_INFO * device) const
{
  if (device == nullptr) {
    return "";
  }

  if (device->nTLayerType ==
      MV_GIGE_DEVICE) {

    return std::string(
      reinterpret_cast<const char *>(
        device->SpecialInfo
          .stGigEInfo
          .chSerialNumber));
  }

  if (device->nTLayerType ==
      MV_USB_DEVICE) {

    return std::string(
      reinterpret_cast<const char *>(
        device->SpecialInfo
          .stUsb3VInfo
          .chSerialNumber));
  }

  return "";
}


// ================================================================
// Convert IP address
// ================================================================

std::string CameraNode::ip_to_string(
  unsigned int ip) const
{
  std::ostringstream stream;

  stream
    << ((ip >> 24) & 0xFF)
    << "."
    << ((ip >> 16) & 0xFF)
    << "."
    << ((ip >> 8) & 0xFF)
    << "."
    << (ip & 0xFF);

  return stream.str();
}

}  // namespace hikrobot_camera
