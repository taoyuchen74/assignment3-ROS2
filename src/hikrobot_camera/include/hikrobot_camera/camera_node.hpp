#ifndef HIKROBOT_CAMERA__CAMERA_NODE_HPP_
#define HIKROBOT_CAMERA__CAMERA_NODE_HPP_

#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "MvCameraControl.h"

namespace hikrobot_camera
{

class CameraNode : public rclcpp::Node
{
public:
  explicit CameraNode(
    const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

  ~CameraNode();

private:
  // Camera lifecycle
  bool initialize_camera();
  bool connect_camera();
  void disconnect_camera();
  bool configure_camera();

  // Image acquisition
  void grab_image();
  void reconnect_camera();
  void publish_image(const MV_FRAME_OUT & frame);

  // Parameters
  rcl_interfaces::msg::SetParametersResult on_parameter_change(
    const std::vector<rclcpp::Parameter> & parameters);

  bool set_float_parameter(
    const std::string & name,
    const std::string & sdk_name,
    double value);

  bool set_frame_rate(double value);
  bool set_pixel_format(const std::string & value);

  // Utilities
  std::string get_serial_number(
    const MV_CC_DEVICE_INFO * device) const;

  std::string ip_to_string(unsigned int ip) const;

  // MVS camera
  void * camera_handle_{nullptr};
  MV_CC_DEVICE_INFO_LIST device_list_{};

  // ROS2
  rclcpp::TimerBase::SharedPtr grab_timer_;
  rclcpp::TimerBase::SharedPtr reconnect_timer_;

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr
    image_publisher_;

  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr
    parameter_callback_handle_;

  // Parameters
  std::string camera_ip_;
  std::string serial_number_;
  std::string image_topic_;
  std::string pixel_format_;

  double exposure_time_{10000.0};
  double gain_{0.0};
  double frame_rate_{30.0};

  // State
  bool connected_{false};
  bool grabbing_{false};
};

}  // namespace hikrobot_camera

#endif  // HIKROBOT_CAMERA__CAMERA_NODE_HPP_