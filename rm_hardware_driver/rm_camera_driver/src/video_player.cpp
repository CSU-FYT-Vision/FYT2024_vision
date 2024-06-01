// Created by Chengfu Zou on 2023.7.2
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

// std
#include <filesystem>
#include <string>
// ros2
#include <camera_info_manager/camera_info_manager.hpp>
#include <image_transport/camera_publisher.hpp>
#include <image_transport/image_transport.hpp>
#include <rclcpp/rate.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/utilities.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>
// OpenCV
#include <opencv2/opencv.hpp>
// project
#include "rm_utils/heartbeat.hpp"
#include "rm_utils/logger/log.hpp"

namespace fyt::camera_driver {
class VideoPlayerNode : public rclcpp::Node {
public:
  explicit VideoPlayerNode(const rclcpp::NodeOptions &options)
  : Node("camera_driver", options), frame_cnt_(0) {
    FYT_REGISTER_LOGGER("camera_driver", "~/fyt2024-log", INFO);
    FYT_INFO("camera_driver", "Starting VideoPlayerNode!");
    // Get parameters
    video_path = this->declare_parameter("path", "/home/zcf/Downloads/rune.avi");
    std::string camera_info_url =
      this->declare_parameter("camera_info_url", "package://rm_bringup/config/camera_info.yaml");
    std::string frame_id = this->declare_parameter("frame_id", "camera_optical_frame");
    int frame_rate = this->declare_parameter("frame_rate", 30);
    start_frame_ = this->declare_parameter("start_frame", 0);
    is_loop_ = this->declare_parameter("keep_looping", true);

    // Heartbeat
    heartbeat_ = HeartBeatPublisher::create(this);

    // Open video file
    std::filesystem::path video_file(video_path);

    if (!std::filesystem::exists(video_file)) {
      FYT_ERROR("camera_driver", "Video file {} does not exist!", video_file.string());
      rclcpp::shutdown();
      return;
    }
    cap_.open(video_path);
    if (!cap_.isOpened()) {
      FYT_ERROR("camera_driver", "Video file open failed!");
      rclcpp::shutdown();
      return;
    }

    // Set image msg
    image_msg_ = std::make_shared<sensor_msgs::msg::Image>();
    image_msg_->header.frame_id = frame_id;
    image_msg_->encoding = sensor_msgs::image_encodings::BGR8;
    image_msg_->width = cap_.get(cv::CAP_PROP_FRAME_WIDTH);
    image_msg_->height = cap_.get(cv::CAP_PROP_FRAME_HEIGHT);
    image_msg_->step = image_msg_->width * 3;
    image_msg_->data.resize(image_msg_->step * image_msg_->height);

    // Set camera info
    camera_info_manager_ = std::make_shared<camera_info_manager::CameraInfoManager>(
      this, "camera_driver", "file://" + video_path);
    if (camera_info_manager_->validateURL(camera_info_url)) {
      camera_info_manager_->loadCameraInfo(camera_info_url);
      camera_info_ = camera_info_manager_->getCameraInfo();
    } else {
      camera_info_manager_->setCameraName(video_path);
      sensor_msgs::msg::CameraInfo camera_info;
      camera_info.header.frame_id = "camera_optical_frame";
      camera_info.header.stamp = this->now();
      camera_info.width = image_msg_->width;
      camera_info.height = image_msg_->height;
      camera_info_manager_->setCameraInfo(camera_info);
      FYT_WARN("camera_driver", "Invalid camera info URL: {}", camera_info_url);
    }
    camera_info_.header.frame_id = frame_id;
    camera_info_.header.stamp = this->now();

    // pub
    camera_pub_ = image_transport::create_camera_publisher(this, "image_raw");

    // Loop
    loop_rate_ = std::make_shared<rclcpp::WallRate>(frame_rate);
    timer_ = this->create_wall_timer(std::chrono::milliseconds(1), [this]() {
      cap_ >> frame_;
      if (frame_.empty()) {
        FYT_INFO("camera_driver", "Video file ends!");
        if (!is_loop_) {
          rclcpp::shutdown();
          return;
        } else {
          cap_.open(video_path);
          frame_cnt_ = 0;
          return;
        }
      }
      memcpy(image_msg_->data.data(), frame_.data, image_msg_->step * image_msg_->height);
      frame_cnt_++;
      if (frame_cnt_ < start_frame_) {
        FYT_INFO("camera_driver", "Skipping frame {}", frame_cnt_);
        return;
      }
      image_msg_->header.stamp = camera_info_.header.stamp = this->now();
      camera_pub_.publish(*image_msg_, camera_info_);
      loop_rate_->sleep();
    });

    rclcpp::spin(this->get_node_base_interface());
  }

private:
  HeartBeatPublisher::SharedPtr heartbeat_;
  std::string video_path;
  image_transport::CameraPublisher camera_pub_;
  bool is_loop_;
  cv::VideoCapture cap_;
  cv::Mat frame_;
  sensor_msgs::msg::Image::SharedPtr image_msg_;
  sensor_msgs::msg::CameraInfo camera_info_;
  std::shared_ptr<camera_info_manager::CameraInfoManager> camera_info_manager_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::WallRate::SharedPtr loop_rate_;
  int start_frame_;
  int frame_cnt_;
};
}  // namespace fyt::camera_driver

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(fyt::camera_driver::VideoPlayerNode)
