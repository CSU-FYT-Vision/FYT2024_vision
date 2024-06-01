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

#include "rune_detector/rune_detector_node.hpp"
// ros2
#include <cv_bridge/cv_bridge.h>
#include <rmw/qos_profiles.h>

#include <rclcpp/qos.hpp>
// std
#include <algorithm>
#include <array>
#include <filesystem>
#include <numeric>
#include <vector>
// third party
#include <opencv2/imgproc.hpp>
// project
#include "rm_utils/assert.hpp"
#include "rm_utils/common.hpp"
#include "rm_utils/logger/log.hpp"
#include "rm_utils/url_resolver.hpp"
#include "rune_detector/types.hpp"

namespace fyt::rune {
RuneDetectorNode::RuneDetectorNode(const rclcpp::NodeOptions &options)
: Node("rune_detector", options), is_rune_(false) {
  FYT_REGISTER_LOGGER("rune_detector", "~/fyt2024-log", INFO);
  FYT_INFO("rune_detector", "Starting RuneDetectorNode!");

  frame_id_ = declare_parameter("frame_id", "camera_optical_frame");
  detect_r_tag_ = declare_parameter("detect_r_tag", true);
  binary_thresh_ = declare_parameter("min_lightness", 100);
  requests_limit_ = declare_parameter("requests_limit", 5);

  // Detector
  rune_detector_ = initDetector();
  // Rune Publisher
  rune_pub_ = this->create_publisher<rm_interfaces::msg::RuneTarget>("rune_detector/rune_target",
                                                                     rclcpp::SensorDataQoS());

  // Debug Publishers
  this->debug_ = declare_parameter("debug", true);
  if (this->debug_) {
    createDebugPublishers();
  }
  auto qos = rclcpp::SensorDataQoS();
  qos.keep_last(1);
  img_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
    "image_raw", qos, std::bind(&RuneDetectorNode::imageCallback, this, std::placeholders::_1));
  set_rune_mode_srv_ = this->create_service<rm_interfaces::srv::SetMode>(
    "rune_detector/set_mode",
    std::bind(
      &RuneDetectorNode::setModeCallback, this, std::placeholders::_1, std::placeholders::_2));

  // Heartbeat
  heartbeat_ = HeartBeatPublisher::create(this);
}

std::unique_ptr<RuneDetector> RuneDetectorNode::initDetector() {
  std::string model_path =
    this->declare_parameter("detector.model", "package://rune_detector/model/yolox_rune_3.6m.onnx");
  std::string device_type = this->declare_parameter("detector.device_type", "AUTO");
  FYT_ASSERT(!model_path.empty());
  FYT_INFO("rune_detector", "model : {}, device : {}", model_path, device_type);

  float conf_threshold = this->declare_parameter("detector.confidence_threshold", 0.50);
  int top_k = this->declare_parameter("detector.top_k", 128);
  float nms_threshold = this->declare_parameter("detector.nms_threshold", 0.3);

  namespace fs = std::filesystem;
  fs::path resolved_path = utils::URLResolver::getResolvedPath(model_path);
  FYT_ASSERT_MSG(fs::exists(resolved_path), resolved_path.string() + " Not Found");

  // Set dynamic parameter callback
  rcl_interfaces::msg::SetParametersResult onSetParameters(
    std::vector<rclcpp::Parameter> parameters);
  on_set_parameters_callback_handle_ = this->add_on_set_parameters_callback(
    std::bind(&RuneDetectorNode::onSetParameters, this, std::placeholders::_1));

  // Create detector
  auto rune_detector = std::make_unique<RuneDetector>(
    resolved_path, device_type, conf_threshold, top_k, nms_threshold);
  // Set detect callback
  rune_detector->setCallback(std::bind(&RuneDetectorNode::inferResultCallback,
                                       this,
                                       std::placeholders::_1,
                                       std::placeholders::_2,
                                       std::placeholders::_3));
  // init detector
  rune_detector->init();
  return rune_detector;
}

void RuneDetectorNode::imageCallback(const sensor_msgs::msg::Image::ConstSharedPtr msg) {
  if (is_rune_ == false) {
    return;
  }

  // Limits request size
  while (detect_requests_.size() > static_cast<size_t>(requests_limit_)) {
    detect_requests_.front().get();
    detect_requests_.pop();
  }

  auto timestamp = rclcpp::Time(msg->header.stamp);
  frame_id_ = msg->header.frame_id;
  auto img = cv_bridge::toCvCopy(msg, "rgb8")->image;

  // Push image to detector
  detect_requests_.push(rune_detector_->pushInput(img, timestamp.nanoseconds()));
};

rcl_interfaces::msg::SetParametersResult RuneDetectorNode::onSetParameters(
  std::vector<rclcpp::Parameter> parameters) {
  rcl_interfaces::msg::SetParametersResult result;
  for (const auto &param : parameters) {
    if (param.get_name() == "binary_thresh") {
      binary_thresh_ = param.as_int();
    }
  }
  result.successful = true;
  return result;
}

void RuneDetectorNode::inferResultCallback(std::vector<RuneObject> &objs,
                                           int64_t timestamp_nanosec,
                                           const cv::Mat &src_img) {
  auto timestamp = rclcpp::Time(timestamp_nanosec);
  // Used to draw debug info
  cv::Mat debug_img;
  if (debug_) {
    debug_img = src_img.clone();
  }

  rm_interfaces::msg::RuneTarget rune_msg;
  rune_msg.header.frame_id = frame_id_;
  rune_msg.header.stamp = timestamp;
  rune_msg.is_big_rune = is_big_rune_;

  // Erase all object that not match the color
  objs.erase(
    std::remove_if(objs.begin(),
                   objs.end(),
                   [c = detect_color_](const auto &obj) -> bool { return obj.color != c; }),
    objs.end());

  if (!objs.empty()) {
    // Sort by probability
    std::sort(objs.begin(), objs.end(), [](const RuneObject &a, const RuneObject &b) {
      return a.prob > b.prob;
    });

    cv::Point2f r_tag;
    cv::Mat binary_roi = cv::Mat::zeros(1, 1, CV_8UC3);
    if (detect_r_tag_) {
      // Detect R tag using traditional method
      std::tie(r_tag, binary_roi) =
        rune_detector_->detectRTag(src_img, binary_thresh_, objs.at(0).pts.r_center);
    } else {
      // Use the average center of all objects as the center of the R tag
      r_tag = std::accumulate(objs.begin(),
                              objs.end(),
                              cv::Point2f(0, 0),
                              [n = static_cast<float>(objs.size())](cv::Point2f p, auto &o) {
                                return p + o.pts.r_center / n;
                              });
    }
    // Assign the center of the R tag to all objects
    std::for_each(objs.begin(), objs.end(), [r = r_tag](RuneObject &obj) { obj.pts.r_center = r; });

    // Draw binary roi
    if (debug_ && !debug_img.empty()) {
      cv::Rect roi =
        cv::Rect(debug_img.cols - binary_roi.cols, 0, binary_roi.cols, binary_roi.rows);
      binary_roi.copyTo(debug_img(roi));
      cv::rectangle(debug_img, roi, cv::Scalar(150, 150, 150), 2);
    }

    // The final target is the inactivated rune with the highest probability
    auto result_it =
      std::find_if(objs.begin(), objs.end(), [c = detect_color_](const auto &obj) -> bool {
        return obj.type == RuneType::INACTIVATED && obj.color == c;
      });

    if (result_it != objs.end()) {
      // FYT_DEBUG("rune_detector", "Detected!");
      rune_msg.is_lost = false;
      rune_msg.pts[0].x = result_it->pts.r_center.x;
      rune_msg.pts[0].y = result_it->pts.r_center.y;
      rune_msg.pts[1].x = result_it->pts.bottom_left.x;
      rune_msg.pts[1].y = result_it->pts.bottom_left.y;
      rune_msg.pts[2].x = result_it->pts.top_left.x;
      rune_msg.pts[2].y = result_it->pts.top_left.y;
      rune_msg.pts[3].x = result_it->pts.top_right.x;
      rune_msg.pts[3].y = result_it->pts.top_right.y;
      rune_msg.pts[4].x = result_it->pts.bottom_right.x;
      rune_msg.pts[4].y = result_it->pts.bottom_right.y;
    } else {
      // All runes are activated
      rune_msg.is_lost = true;
    }
  } else {
    // All runes are not the target color
    rune_msg.is_lost = true;
  }

  rune_pub_->publish(std::move(rune_msg));

  if (debug_) {
    if (debug_img.empty()) {
      // Avoid debug_mode change in processing
      return;
    }

    // Draw detection result
    for (auto &obj : objs) {
      auto pts = obj.pts.toVector2f();
      cv::Point2f aim_point = std::accumulate(pts.begin() + 1, pts.end(), cv::Point2f(0, 0)) / 4;

      cv::Scalar line_color =
        obj.type == RuneType::INACTIVATED ? cv::Scalar(50, 255, 50) : cv::Scalar(255, 50, 255);
      cv::putText(debug_img,
                  fmt::format("{:.2f}", obj.prob),
                  cv::Point2i(pts[1]),
                  cv::FONT_HERSHEY_SIMPLEX,
                  0.8,
                  line_color,
                  2);
      cv::polylines(debug_img, obj.pts.toVector2i(), true, line_color, 2);
      cv::circle(debug_img, aim_point, 5, line_color, -1);

      std::string rune_type = obj.type == RuneType::INACTIVATED ? "_HIT" : "_OK";
      std::string rune_color = enemyColorToString(obj.color);
      cv::putText(debug_img,
                  rune_color + rune_type,
                  cv::Point2i(pts[2]),
                  cv::FONT_HERSHEY_SIMPLEX,
                  0.8,
                  line_color,
                  2);
    }

    auto end = this->get_clock()->now();
    auto duration = end.seconds() - timestamp.seconds();
    std::string letency = fmt::format("Latency: {:.3f}ms", duration * 1000);
    cv::putText(debug_img,
                letency,
                cv::Point2i(10, 30),
                cv::FONT_HERSHEY_SIMPLEX,
                0.8,
                cv::Scalar(0, 255, 255),
                2);
    result_img_pub_.publish(cv_bridge::CvImage(rune_msg.header, "rgb8", debug_img).toImageMsg());
  }
}

void RuneDetectorNode::setModeCallback(
  const std::shared_ptr<rm_interfaces::srv::SetMode::Request> request,
  std::shared_ptr<rm_interfaces::srv::SetMode::Response> response) {
  response->success = true;

  VisionMode mode = static_cast<VisionMode>(request->mode);
  std::string mode_name = visionModeToString(mode);
  if (mode_name == "UNKNOWN") {
    FYT_ERROR("rune_detector", "Invalid mode: {}", request->mode);
    return;
  }

  auto createImageSub = [this]() {
    if (img_sub_ == nullptr) {
      img_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
        "image_raw",
        rclcpp::SensorDataQoS(),
        std::bind(&RuneDetectorNode::imageCallback, this, std::placeholders::_1));
    }
  };

  switch (mode) {
    case VisionMode::SMALL_RUNE_RED: {
      is_rune_ = true;
      is_big_rune_ = false;
      detect_color_ = EnemyColor::RED;
      createImageSub();
      break;
    }
    case VisionMode::SMALL_RUNE_BLUE: {
      is_rune_ = true;
      is_big_rune_ = false;
      detect_color_ = EnemyColor::BLUE;
      createImageSub();
      break;
    }
    case VisionMode::BIG_RUNE_RED: {
      is_rune_ = true;
      is_big_rune_ = true;
      detect_color_ = EnemyColor::RED;
      createImageSub();
      break;
    }
    case VisionMode::BIG_RUNE_BLUE: {
      is_rune_ = true;
      is_big_rune_ = true;
      detect_color_ = EnemyColor::BLUE;
      createImageSub();
      break;
    }
    default: {
      is_rune_ = false;
      is_big_rune_ = false;
      img_sub_.reset();
      break;
    }
  }

  FYT_WARN("rune_detector", "Set Rune Mode: {}", visionModeToString(mode));
}

void RuneDetectorNode::createDebugPublishers() {
  result_img_pub_ = image_transport::create_publisher(this, "rune_detector/result_img");
}

void RuneDetectorNode::destroyDebugPublishers() { result_img_pub_.shutdown(); }

}  // namespace fyt::rune
#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(fyt::rune::RuneDetectorNode)
