#ifndef HIKROBOT_CAMERA__CAMERA_NODE_HPP_
#define HIKROBOT_CAMERA__CAMERA_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <vector>
#include <string>

namespace hikrobot_camera
{

class CameraNode : public rclcpp::Node
{
public:
  explicit CameraNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
  ~CameraNode() override;

private:
  // 参数回调
  rcl_interfaces::msg::SetParametersResult parameters_callback(
    const std::vector<rclcpp::Parameter> & parameters);

  // 取图与发布
  void grab_and_publish();

  // 相机生命周期
  bool init_camera();
  void close_camera();

  // 参数设置辅助
  bool set_camera_float_param(const std::string & name, double value);
  bool set_camera_enum_param(const std::string & name, const std::string & value);

  // ROS 2 资源
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;

  // 相机资源与状态
  void * handle_ = nullptr;
  bool is_camera_ready_ = false;
  int lost_frames_ = 0;
};

}  // namespace hikrobot_camera

#endif  // HIKROBOT_CAMERA__CAMERA_NODE_HPP_