// Created by Chengfu Zou on 2023.7.1
// Copyright (C) FYT Vision Group. All rights reserved.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "rm_camera_driver/daheng_camera.hpp"
// std
#include <chrono>
#include <cstdlib>
#include <ctime>
#include <memory>
#include <stdexcept>
#include <thread>
// ros2
#include <rclcpp/rate.hpp>
#include <rclcpp/utilities.hpp>
// OpenCV
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>
// Daheng Galaxy Driver
#include "daheng/GxIAPI.h"
// project
#include "rm_utils/logger/log.hpp"

// Callback Wrapper for adapting GxAPI's interface;
static std::function<void GX_STDC(GX_FRAME_CALLBACK_PARAM *)> g_callback;
extern "C" void GX_STDC CallbackWrapper(GX_FRAME_CALLBACK_PARAM *arg) {
  if (g_callback) {
    g_callback(arg);
  }
}

namespace fyt::camera_driver {
DahengCameraNode::DahengCameraNode(const rclcpp::NodeOptions &options)
: Node("camera_driver", options) {
  FYT_REGISTER_LOGGER("camera_driver", "~/fyt2024-log", INFO);
  FYT_INFO("camera_driver", "Starting DahengCameraNode!");

  camera_name_ = this->declare_parameter("camera_name", "daheng");
  camera_info_url_ =
    this->declare_parameter("camera_info_url", "package://rm_bringup/config/camera_info.yaml");
  frame_id_ = this->declare_parameter("camera_frame_id", "camera_optical_frame");
  pixel_format_ = this->declare_parameter("pixel_format", "rgb8");
  resolution_width_ = this->declare_parameter("resolution_width", 1280);
  resolution_height_ = this->declare_parameter("resolution_height", 1024);
  auto_white_balance_ = this->declare_parameter("auto_white_balance", 1);
  frame_rate_ = this->declare_parameter("frame_rate", 210);
  exposure_time_ = this->declare_parameter("exposure_time", 2000);
  gain_ = this->declare_parameter("gain", 5.0);
  offest_x_ = this->declare_parameter("offsetX", 0);
  offset_y_ = this->declare_parameter("offsetY", 0);
  //为存储原始图像数据申请空间
  image_msg_.header.frame_id = frame_id_;
  image_msg_.encoding = pixel_format_;
  image_msg_.height = resolution_height_;
  image_msg_.width = resolution_width_;
  image_msg_.step = resolution_width_ * 3;
  image_msg_.data.resize(image_msg_.height * image_msg_.step);

  if (pixel_format_ == "mono8") {
    gx_pixel_format_ = GX_PIXEL_FORMAT_MONO8;
  } else if (pixel_format_ == "mono16") {
    gx_pixel_format_ = GX_PIXEL_FORMAT_MONO16;
  } else if (pixel_format_ == "bgr8") {
    gx_pixel_format_ = GX_PIXEL_FORMAT_BGR8;
    gx_bayer_type_ = BAYERBG;
  } else if (pixel_format_ == "rgb8") {
    gx_pixel_format_ = GX_PIXEL_FORMAT_RGB8;
    gx_bayer_type_ = BAYERRG;
  } else if (pixel_format_ == "bgra8") {
    gx_pixel_format_ = GX_PIXEL_FORMAT_BGRA8;
  } else {
    FYT_ERROR("camera_driver", "Unsupported pixel format: {}", pixel_format_);
  }

  camera_info_manager_ =
    std::make_unique<camera_info_manager::CameraInfoManager>(this, camera_name_);

  if (camera_info_manager_->validateURL(camera_info_url_)) {
    camera_info_manager_->loadCameraInfo(camera_info_url_);
    camera_info_ = camera_info_manager_->getCameraInfo();
  } else {
    camera_info_manager_->setCameraName(camera_name_);
    sensor_msgs::msg::CameraInfo camera_info;
    camera_info.width = image_msg_.width;
    camera_info.height = image_msg_.height;
    camera_info_manager_->setCameraInfo(camera_info);
    FYT_WARN("camera_driver", "Invalid camera info URL: {}", camera_info_url_);
  }
  camera_info_.header.frame_id = frame_id_;
  camera_info_.header.stamp = this->now();

  bool use_sensor_data_qos = this->declare_parameter("use_sensor_data_qos", true);
  auto qos = use_sensor_data_qos ? rmw_qos_profile_sensor_data : rmw_qos_profile_default;
  pub_ = image_transport::create_camera_publisher(this, "image_raw", qos);

  // Heartbeat
  heartbeat_ = HeartBeatPublisher::create(this);

  // Check if camera is alive every seconds
  timer_ = this->create_wall_timer(std::chrono::milliseconds(1000),
                                   std::bind(&DahengCameraNode::timerCallback, this));

  // Param set callback
  on_set_parameters_callback_handle_ = this->add_on_set_parameters_callback(
    std::bind(&DahengCameraNode::onSetParameters, this, std::placeholders::_1));

  // Recorder
  bool enable_recorder = this->declare_parameter("recording", false);
  if (enable_recorder) {
    std::string home = std::getenv("HOME");

    namespace fs = std::filesystem;
    std::filesystem::path video_path = fs::path(home) / "fyt2024-log/video/" /
                                       std::string(std::to_string(std::time(nullptr)) + ".avi");

    recorder_ = std::make_unique<Recorder>(
      video_path, frame_rate_, cv::Size(resolution_width_, resolution_height_));
    recorder_->start();
    FYT_INFO("camera_driver", "Recorder started! Video file: {}", video_path.string());
  }

  FYT_INFO("camera_driver", "DahengCameraNode has been initialized!");
}

DahengCameraNode::~DahengCameraNode() {
  close();
  if (recorder_ != nullptr) {
    recorder_->stop();
    FYT_INFO(
      "camera_driver", "Recorder stopped! Video file {} has been saved", recorder_->path.string());
  }
  FYT_INFO("camera_driver", "DahengCameraNode has been destroyed!");
}

void DahengCameraNode::timerCallback() {
  // Open camera
  while (!is_open_ && rclcpp::ok()) {
    bool is_open_success = open();
    if (!is_open_success) {
      FYT_ERROR("camera_driver", "open() failed");
      close();
      return;
    }
  }
  // Watch Dog
  double dt = (this->now() - rclcpp::Time(camera_info_.header.stamp)).seconds();
  if (dt > 5.0) {
    FYT_WARN("camera_driver", "Camera is not alive! lost frame for {:.2f} seconds", dt);
    close();
  }
}

void DahengCameraNode::close() {
  FYT_INFO("camera_driver", "Closing Daheng Galaxy Camera Device!");
  if (is_open_ && dev_handle_ != nullptr) {
    //发送停采命令
    GXStreamOff(dev_handle_);
    GXCloseDevice(dev_handle_);
    // GXUnregisterCaptureCallback(dev_handle_);
  }
  GXCloseLib();
  is_open_ = false;
}

bool DahengCameraNode::open() {
  FYT_INFO("camera_driver", "Opening Daheng Galaxy Camera Device!");
  if (is_open_) {
    FYT_WARN("camera_driver", "Daheng Galaxy Camera Device is already opened!");
    close();
  }
  gx_status_ = GX_STATUS_SUCCESS;
  GX_OPEN_PARAM openParam;
  uint32_t device_num = 0;
  openParam.accessMode = GX_ACCESS_EXCLUSIVE;
  openParam.openMode = GX_OPEN_INDEX;
  openParam.pszContent = (char *)"1";
  // 尝试初始化库
  gx_status_ = GXInitLib();
  if (gx_status_ != GX_STATUS_SUCCESS) {
    FYT_ERROR("camera_driver", "Can't init lib");
    return false;
  }

  // 枚举设备列表
  gx_status_ = GXUpdateDeviceList(&device_num, 1000);
  if ((gx_status_ != GX_STATUS_SUCCESS) || (device_num <= 0)) {
    FYT_WARN("camera_driver", "Can't find camera");
    return false;
  }
  FYT_INFO("camera_driver", "Found {} devices", device_num);
  //打开设备
  gx_status_ = GXOpenDevice(&openParam, &dev_handle_);
  if (gx_status_ != GX_STATUS_SUCCESS) {
    FYT_ERROR("camera_driver", "Can't open device");
    return false;
  }
  is_open_ = true;

  GXGetInt(dev_handle_, GX_INT_WIDTH_MAX, &max_resolution_width_);
  GXGetInt(dev_handle_, GX_INT_HEIGHT_MAX, &max_resolution_height_);

  // 设置像素格式
  GXSetEnum(dev_handle_, GX_ENUM_PIXEL_FORMAT, gx_pixel_format_);
  // 设置宽度
  GXSetInt(dev_handle_, GX_INT_WIDTH, resolution_width_);
  // 设置高度
  GXSetInt(dev_handle_, GX_INT_HEIGHT, resolution_height_);

  // // 从中心裁剪
  // int64_t offest_x_ = (MAX_RESOLUTION_WIDTH - resolution_width_) / 2;
  // int64_t offset_y_ = (MAX_RESOLUTION_HEIGHT - resolution_height_) / 2;
  //  GXSetEnum(dev_handle_, GX_ENUM_RREGION_SELECTOR,
  //  GX_REGION_SELECTOR_REGION0);
  GXSetInt(dev_handle_, GX_INT_OFFSET_X, offest_x_);
  GXSetInt(dev_handle_, GX_INT_OFFSET_Y, offset_y_);

  // 获取图像大小
  GXGetInt(dev_handle_, GX_INT_PAYLOAD_SIZE, &gx_pay_load_size_);
  //设置是否开启自动白平衡
  if (auto_white_balance_) {
    GXSetEnum(dev_handle_, GX_ENUM_BALANCE_WHITE_AUTO, 1);
  } else {
    //设置白平衡数值  如果开启自动白平衡则无效
    GXSetEnum(dev_handle_, GX_ENUM_LIGHT_SOURCE_PRESET, GX_LIGHT_SOURCE_PRESET_DAYLIGHT_5000K);
  }
  //设置帧率
  GXSetEnum(dev_handle_, GX_ENUM_ACQUISITION_FRAME_RATE_MODE, GX_ENUM_COVER_FRAMESTORE_MODE_ON);
  GXSetFloat(dev_handle_, GX_FLOAT_ACQUISITION_FRAME_RATE, frame_rate_);

  //设置曝光时间
  GXSetFloat(dev_handle_, GX_FLOAT_EXPOSURE_TIME, exposure_time_);
  //设置增益
  GXSetFloat(dev_handle_, GX_FLOAT_GAIN, gain_);

  //设置采集模式连续采集
  GXSetEnum(dev_handle_, GX_ENUM_ACQUISITION_MODE, GX_ACQ_MODE_CONTINUOUS);
  GXSetInt(dev_handle_, GX_INT_ACQUISITION_SPEED_LEVEL, 1);

  //注册图像处理回调函数
  // void (*)(GX_FRAME_CALLBACK_PARAM *)
  g_callback = std::bind(&DahengCameraNode::onFrameCallbackFun, this, std::placeholders::_1);
  gx_status_ = GXRegisterCaptureCallback(dev_handle_, nullptr, CallbackWrapper);
  if (gx_status_ != GX_STATUS_SUCCESS) {
    FYT_ERROR("camera_driver", "Register capture callback function failed!");
    return false;
  }

  //发送开采命令
  gx_status_ = GXStreamOn(dev_handle_);
  if (gx_status_ != GX_STATUS_SUCCESS) {
    FYT_ERROR("camera_driver", "Send start capture command failed!");
    return false;
  }
  FYT_INFO("camera_driver", "Daheng Galaxy Camera Device Open Success!");
  return true;
}

void GX_STDC DahengCameraNode::onFrameCallbackFun(GX_FRAME_CALLBACK_PARAM *pFrame) {
  if (pFrame->status == GX_FRAME_STATUS_SUCCESS) {
    // RGB转换
    DxRaw8toRGB24((void *)pFrame->pImgBuf,
                  image_msg_.data.data(),
                  pFrame->nWidth,
                  pFrame->nHeight,
                  RAW2RGB_NEIGHBOUR,
                  static_cast<DX_PIXEL_COLOR_FILTER>(gx_bayer_type_),
                  false);
    image_msg_.header.stamp = camera_info_.header.stamp = this->now();
    pub_.publish(image_msg_, camera_info_);
    if (recorder_ != nullptr) {
      recorder_->addFrame(image_msg_.data);
    }
  }
}

rcl_interfaces::msg::SetParametersResult DahengCameraNode::onSetParameters(
  std::vector<rclcpp::Parameter> parameters) {
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  for (const auto &param : parameters) {
    if (param.get_name() == "exposure_time") {
      exposure_time_ = param.as_int();
      GXSetFloat(dev_handle_, GX_FLOAT_EXPOSURE_TIME, static_cast<double>(exposure_time_));
      FYT_INFO("camera_driver", "Set exposure_time: {}", exposure_time_);
    } else if (param.get_name() == "gain") {
      gain_ = param.as_double();
      GXSetFloat(dev_handle_, GX_FLOAT_GAIN, gain_);
      FYT_INFO("camera_driver", "Set gain: {}", gain_);
    }
  }
  return result;
}

}  // namespace fyt::camera_driver

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(fyt::camera_driver::DahengCameraNode)
