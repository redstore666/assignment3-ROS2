#include "hikrobot_camera/camera_node.hpp"

#include <chrono>
#include <cstring>
#include <memory>
#include <vector>

#include "MvCameraControl.h"

using namespace std::chrono_literals;

namespace hikrobot_camera
{

  // ============ 构造函数 ============
  CameraNode::CameraNode(const rclcpp::NodeOptions &options)
      : Node("hikrobot_camera", options)
  {
    // 1. 声明参数
    this->declare_parameter("exposure_time", 5000.0);
    this->declare_parameter("gain", 10.0);
    this->declare_parameter("frame_rate", 30.0);
    this->declare_parameter("pixel_format", "BayerRG8");
    this->declare_parameter("camera_serial", "");

    // 2. 注册参数回调
    param_callback_handle_ = this->add_on_set_parameters_callback(
        std::bind(&CameraNode::parameters_callback, this, std::placeholders::_1));

    // 3. 创建发布者
    publisher_ = this->create_publisher<sensor_msgs::msg::Image>("/image_raw", 10);

    // 4. 尝试初始化相机
    if (init_camera())
    {
      is_camera_ready_ = true;
    }
    else
    {
      RCLCPP_WARN(this->get_logger(), "相机未连接，等待相机接入...");
    }

    // 5. 创建定时器
    timer_ = this->create_wall_timer(33ms, std::bind(&CameraNode::grab_and_publish, this));
    RCLCPP_INFO(this->get_logger(), "真实相机节点已启动。");
  }

  // ============ 析构函数 ============
  CameraNode::~CameraNode()
  {
    close_camera();
  }

  // ============ 参数回调 ============
  rcl_interfaces::msg::SetParametersResult CameraNode::parameters_callback(
      const std::vector<rclcpp::Parameter> &parameters)
  {
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;

    for (const auto &param : parameters)
    {
      if (!is_camera_ready_)
      {
        result.successful = false;
        result.reason = "相机未连接，无法设置参数";
        continue;
      }

      bool set_ok = true;
      if (param.get_name() == "exposure_time")
      {
        MV_CC_SetEnumValue(handle_, "ExposureAuto", 0);
        MV_CC_StopGrabbing(handle_);
        set_ok = set_camera_float_param("ExposureTime", param.as_double());
        MV_CC_StartGrabbing(handle_);
      }
      else if (param.get_name() == "gain")
      {
        MV_CC_SetEnumValue(handle_, "GainAuto", 0);
        MV_CC_StopGrabbing(handle_);
        set_ok = set_camera_float_param("Gain", param.as_double());
        MV_CC_StartGrabbing(handle_);
      }
      else if (param.get_name() == "frame_rate")
      {
        MV_CC_StopGrabbing(handle_);
        set_ok = set_camera_float_param("AcquisitionFrameRate", param.as_double());
        MV_CC_StartGrabbing(handle_);
      }
      else if (param.get_name() == "pixel_format")
      {
        MV_CC_StopGrabbing(handle_);
        set_ok = set_camera_enum_param("PixelFormat", param.as_string());
        MV_CC_StartGrabbing(handle_);
      }

      if (!set_ok)
      {
        result.successful = false;
        result.reason = param.get_name() + " 设置失败，SDK 返回错误";
      }
    }
    return result;
  }

  // ============ 参数写入辅助 ============
  bool CameraNode::set_camera_float_param(const std::string &name, double value)
  {
    int ret = MV_CC_SetFloatValue(handle_, name.c_str(), static_cast<float>(value));
    if (ret != MV_OK)
    {
      RCLCPP_ERROR(this->get_logger(), "设置 %s 失败，错误码: 0x%x", name.c_str(), ret);
      return false;
    }
    return true;
  }

  bool CameraNode::set_camera_enum_param(const std::string &name, const std::string &value)
  {
    int enum_val = 0;
    if (value == "BayerRG8")
      enum_val = PixelType_Gvsp_BayerRG8;
    else if (value == "RGB8")
      enum_val = PixelType_Gvsp_RGB8_Packed;
    else if (value == "Mono8")
      enum_val = PixelType_Gvsp_Mono8;
    else
    {
      RCLCPP_ERROR(this->get_logger(), "不支持的像素格式: %s", value.c_str());
      return false;
    }
    int ret = MV_CC_SetEnumValue(handle_, name.c_str(), enum_val);
    return ret == MV_OK;
  }

  // ============ 相机初始化 ============
  bool CameraNode::init_camera()
  {
    close_camera();

    MV_CC_DEVICE_INFO_LIST device_list;
    memset(&device_list, 0, sizeof(MV_CC_DEVICE_INFO_LIST));
    int ret = MV_CC_EnumDevices(MV_GIGE_DEVICE | MV_USB_DEVICE, &device_list);
    if (ret != MV_OK)
    {
      RCLCPP_ERROR(this->get_logger(), "枚举设备失败，错误码: 0x%x", ret);
      return false;
    }
    if (device_list.nDeviceNum == 0)
    {
      RCLCPP_WARN(this->get_logger(), "未发现任何设备，检查相机连接或IP配置。");
      return false;
    }
    RCLCPP_INFO(this->get_logger(), "发现 %d 个相机设备。", device_list.nDeviceNum);

    std::string target_serial = this->get_parameter("camera_serial").as_string();
    bool found = false;
    int target_index = 0;

    for (unsigned int i = 0; i < device_list.nDeviceNum; i++)
    {
      MV_CC_DEVICE_INFO *info = device_list.pDeviceInfo[i];
      if (info->nTLayerType == MV_GIGE_DEVICE)
      {
        std::string serial((char *)info->SpecialInfo.stGigEInfo.chSerialNumber);
        if (target_serial.empty() || serial == target_serial)
        {
          found = true;
          target_index = i;
          break;
        }
      }
      else if (info->nTLayerType == MV_USB_DEVICE)
      {
        std::string serial((char *)info->SpecialInfo.stUsb3VInfo.chSerialNumber);
        if (target_serial.empty() || serial == target_serial)
        {
          found = true;
          target_index = i;
          break;
        }
      }
    }

    if (!found)
    {
      RCLCPP_ERROR(this->get_logger(), "未找到序列号为 %s 的相机", target_serial.c_str());
      return false;
    }

    ret = MV_CC_CreateHandle(&handle_, device_list.pDeviceInfo[target_index]);
    if (ret != MV_OK)
    {
      RCLCPP_ERROR(this->get_logger(), "创建句柄失败，错误码: 0x%x", ret);
      return false;
    }
    ret = MV_CC_OpenDevice(handle_);
    if (ret != MV_OK)
    {
      RCLCPP_ERROR(this->get_logger(), "打开设备失败，错误码: 0x%x", ret);
      close_camera();
      return false;
    }

    MV_CC_SetEnumValue(handle_, "TriggerMode", MV_TRIGGER_MODE_OFF);

    MV_CC_SetEnumValue(handle_, "ExposureAuto", 0);
    MV_CC_SetEnumValue(handle_, "GainAuto", 0);
    set_camera_float_param("ExposureTime", this->get_parameter("exposure_time").as_double());
    set_camera_float_param("Gain", this->get_parameter("gain").as_double());
    set_camera_float_param("AcquisitionFrameRate", this->get_parameter("frame_rate").as_double());
    set_camera_enum_param("PixelFormat", this->get_parameter("pixel_format").as_string());

    ret = MV_CC_StartGrabbing(handle_);
    if (ret != MV_OK)
    {
      RCLCPP_ERROR(this->get_logger(), "开始取流失败，错误码: 0x%x", ret);
      close_camera();
      return false;
    }

    RCLCPP_INFO(this->get_logger(), "相机连接成功，开始取流。");
    return true;
  }

  // ============ 关闭相机 ============
  void CameraNode::close_camera()
  {
    if (handle_)
    {
      MV_CC_StopGrabbing(handle_);
      MV_CC_CloseDevice(handle_);
      MV_CC_DestroyHandle(handle_);
      handle_ = nullptr;
    }
  }

  // ============ 取图与发布 ============
  void CameraNode::grab_and_publish()
  {
    if (!is_camera_ready_ || !handle_)
    {
      lost_frames_++;
      if (lost_frames_ > 5)
      {
        RCLCPP_INFO(this->get_logger(), "尝试重新连接相机...");
        if (init_camera())
        {
          is_camera_ready_ = true;
          lost_frames_ = 0;
          RCLCPP_INFO(this->get_logger(), "相机重连成功！");
        }
        else
        {
          RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "重连失败，稍后继续尝试...");
          lost_frames_ = 0;
        }
      }
      return;
    }

    MV_FRAME_OUT_INFO_EX frame_info;
    memset(&frame_info, 0, sizeof(MV_FRAME_OUT_INFO_EX));
    std::vector<unsigned char> raw_data(4096 * 3000 * 3);

    int ret = MV_CC_GetOneFrameTimeout(handle_, raw_data.data(), raw_data.size(), &frame_info, 100);

    if (ret == MV_OK)
    {
      lost_frames_ = 0;

      auto msg = sensor_msgs::msg::Image();
      msg.height = frame_info.nHeight;
      msg.width = frame_info.nWidth;
      msg.header.stamp = this->now();
      msg.header.frame_id = "camera_link";
      msg.is_bigendian = 0;

      MV_CC_PIXEL_CONVERT_PARAM cvt_param;
      memset(&cvt_param, 0, sizeof(cvt_param));
      std::vector<unsigned char> rgb_data(4096 * 3000 * 3);

      cvt_param.nWidth = frame_info.nWidth;
      cvt_param.nHeight = frame_info.nHeight;
      cvt_param.pSrcData = raw_data.data();
      cvt_param.nSrcDataLen = frame_info.nFrameLen;
      cvt_param.enSrcPixelType = frame_info.enPixelType;
      cvt_param.enDstPixelType = PixelType_Gvsp_RGB8_Packed;
      cvt_param.pDstBuffer = rgb_data.data();
      cvt_param.nDstBufferSize = rgb_data.size();

      int cvt_ret = MV_CC_ConvertPixelType(handle_, &cvt_param);
      if (cvt_ret == MV_OK)
      {
        msg.encoding = "rgb8";
        msg.step = frame_info.nWidth * 3;
        msg.data.assign(rgb_data.begin(), rgb_data.begin() + (frame_info.nWidth * frame_info.nHeight * 3));
        publisher_->publish(msg);
      }
      else
      {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "像素格式转换失败，错误码: 0x%x", cvt_ret);
      }
    }
    else
    {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "取图失败，错误码: 0x%x", ret);
      lost_frames_++;

      if (lost_frames_ > 5)
      {
        RCLCPP_ERROR(this->get_logger(), "相机连接丢失，尝试重连...");
        close_camera();
        is_camera_ready_ = false;
        lost_frames_ = 0;
      }
    }
  }

} // namespace hikrobot_camera