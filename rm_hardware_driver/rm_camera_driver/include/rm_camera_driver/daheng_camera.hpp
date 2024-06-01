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

#ifndef RM_CAMERA_DRIVER_DAHENG_CAMERA_HPP_
#define RM_CAMERA_DRIVER_DAHENG_CAMERA_HPP_

// std
#include <memory>
// ros2
#include <camera_info_manager/camera_info_manager.hpp>
#include <image_transport/image_transport.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
// Daheng Galaxy Driver
#include "daheng/DxImageProc.h"
#include "daheng/GxIAPI.h"
// project
#include "rm_camera_driver/recorder.hpp"
#include "rm_utils/logger/log.hpp"
#include "rm_utils/heartbeat.hpp"

namespace fyt::camera_driver {

class DahengCameraNode : public rclcpp::Node {
public:
  explicit DahengCameraNode(const rclcpp::NodeOptions &options);

  ~DahengCameraNode();

  bool open();
  void close();

  rclcpp::Time getLatestFrameStamp();

private:
  // Watch dog
  void timerCallback();
  rclcpp::TimerBase::SharedPtr timer_;

  // Heartbeat
  HeartBeatPublisher::SharedPtr heartbeat_;

  // Param set callback
  rcl_interfaces::msg::SetParametersResult onSetParameters(
    std::vector<rclcpp::Parameter> parameters);
  rclcpp::Node::OnSetParametersCallbackHandle::SharedPtr on_set_parameters_callback_handle_;

  // onFrameCallback
  void GX_STDC onFrameCallbackFun(GX_FRAME_CALLBACK_PARAM *pFrame);

  image_transport::CameraPublisher pub_;
  sensor_msgs::msg::Image image_msg_;
  sensor_msgs::msg::CameraInfo camera_info_;
  std::unique_ptr<camera_info_manager::CameraInfoManager> camera_info_manager_;
  std::string camera_name_, camera_info_url_, pixel_format_, frame_id_;

  // Daheng Galaxy API
  GX_DEV_HANDLE dev_handle_;
  GX_STATUS gx_status_;
  int64_t gx_pixel_format_;
  int64_t gx_pay_load_size_;
  int64_t gx_bayer_type_;
  int64_t max_resolution_width_;
  int64_t max_resolution_height_;

  // Recorder
  std::unique_ptr<Recorder> recorder_;

  // General
  bool is_open_ = false;
  int resolution_width_;
  int resolution_height_;
  int auto_white_balance_;
  int frame_rate_;
  int exposure_time_;
  double gain_;
  int offest_x_;
  int offset_y_;
};

}  // namespace fyt::camera_driver
#endif  // endif RM_CAMERA_DRIVER_DAHENG_CAMERA_HPP_
