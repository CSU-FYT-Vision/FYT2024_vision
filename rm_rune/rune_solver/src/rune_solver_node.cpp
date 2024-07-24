// Maintained by Chengfu Zou, Labor
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

#include "rune_solver/rune_solver_node.hpp"
// ros2
#include <cv_bridge/cv_bridge.h>
#include <rmw/qos_profiles.h>

#include <rclcpp/qos.hpp>
// third party
#include <opencv2/calib3d.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/imgproc.hpp>
// project
#include "rm_utils/common.hpp"
#include "rm_utils/logger/log.hpp"
#include "rm_utils/math/pnp_solver.hpp"
#include "rune_solver/motion_model.hpp"

namespace fyt::rune {
RuneSolverNode::RuneSolverNode(const rclcpp::NodeOptions &options) : Node("rune_solver", options) {
  FYT_REGISTER_LOGGER("rune_solver", "~/fyt2024-log", INFO);
  FYT_INFO("rune_solver", "Starting RuneSolverNode!");

  predict_offset_ = declare_parameter("predict_time", 0.1);

  // Tf2 info
  tf2_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
  tf2_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf2_buffer_);

  // RuneSolver
  auto rune_solver_params = RuneSolver::RuneSolverParams{
    .compensator_type = declare_parameter("compensator_type", "ideal"),
    .gravity = declare_parameter("gravity", 9.8),
    .bullet_speed = declare_parameter("bullet_speet", 28.0),
    .angle_offset_thres = declare_parameter("angle_offset_thres", 0.78),
    .lost_time_thres = declare_parameter("lost_time_thres", 0.5),
    .auto_type_determined = declare_parameter("auto_type_determined", true),
  };
  rune_solver_ = std::make_unique<RuneSolver>(rune_solver_params, tf2_buffer_);
  
  // Init manual compensator
  auto angle_offset = declare_parameter("angle_offset", std::vector<std::string>{});
  rune_solver_->manual_compensator->updateMapFlow(angle_offset);

  // EKF for filtering the position of R tag
  // state: x, y, z, yaw
  // measurement: x, y, z, yaw
  // f - Process function
  auto f = Predict();
  // h - Observation function
  auto h = Measure();
  // update_Q - process noise covariance matrix
  std::vector<double> q_vec =
    declare_parameter("ekf.q", std::vector<double>{0.001, 0.001, 0.001, 0.001});
  auto u_q = [q_vec]() {
    Eigen::Matrix<double, X_N, X_N> q = Eigen::MatrixXd::Zero(4, 4);
    q.diagonal() << q_vec[0], q_vec[1], q_vec[2], q_vec[3];
    return q;
  };
  // update_R - measurement noise covariance matrix
  std::vector<double> r_vec = declare_parameter("ekf.r", std::vector<double>{0.1, 0.1, 0.1, 0.1});
  auto u_r = [r_vec](const Eigen::Matrix<double, Z_N, 1> &z) {
    Eigen::Matrix<double, Z_N, Z_N> r = Eigen::MatrixXd::Zero(4, 4);
    r.diagonal() << r_vec[0], r_vec[1], r_vec[2], r_vec[3];
    return r;
  };
  // P - error estimate covariance matrix
  Eigen::MatrixXd p0 = Eigen::MatrixXd::Identity(4, 4);
  rune_solver_->ekf = std::make_unique<RuneCenterEKF>(f, h, u_q, u_r, p0);

  // Target subscriber
  rune_target_sub_ = this->create_subscription<rm_interfaces::msg::RuneTarget>(
    "rune_detector/rune_target",
    rclcpp::SensorDataQoS(),
    std::bind(&RuneSolverNode::runeTargetCallback, this, std::placeholders::_1));

  // Publisher
  gimbal_pub_ = this->create_publisher<rm_interfaces::msg::GimbalCmd>("rune_solver/cmd_gimbal",
                                                                      rclcpp::SensorDataQoS());
  target_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>(
    "rune_solver/predict_target", rclcpp::SensorDataQoS());

  // Set dynamic parameter callback
  on_set_parameters_callback_handle_ = this->add_on_set_parameters_callback(
    std::bind(&RuneSolverNode::onSetParameters, this, std::placeholders::_1));

  // Camera info
  cam_info_sub_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
    "camera_info",
    rclcpp::SensorDataQoS(),
    [this](sensor_msgs::msg::CameraInfo::ConstSharedPtr camera_info) {
      cam_info_ = std::make_shared<sensor_msgs::msg::CameraInfo>(*camera_info);
      rune_solver_->pnp_solver = std::make_unique<PnPSolver>(camera_info->k, camera_info->d);
      rune_solver_->pnp_solver->setObjectPoints("rune", RUNE_OBJECT_POINTS);
      cam_info_sub_.reset();
    });

  // Enable/Disable Rune Solver
  set_mode_srv_ = this->create_service<rm_interfaces::srv::SetMode>(
    "rune_solver/set_mode",
    std::bind(
      &RuneSolverNode::setModeCallback, this, std::placeholders::_1, std::placeholders::_2));

  // Debug info
  debug_ = this->declare_parameter("debug", true);
  if (debug_) {
    observed_angle_pub_ = this->create_publisher<rm_interfaces::msg::DebugRuneAngle>(
      "rune_solver/observed_angle", rclcpp::SensorDataQoS());
    predicted_angle_pub_ = this->create_publisher<rm_interfaces::msg::DebugRuneAngle>(
      "rune_solver/predicted_angle", rclcpp::SensorDataQoS());
    fitter_text_pub_ = this->create_publisher<std_msgs::msg::String>("rune_solver/fitting_info",
                                                                     rclcpp::SensorDataQoS());
    // Marker
    r_tag_pos_marker_.ns = "r_tag_position";
    r_tag_pos_marker_.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    r_tag_pos_marker_.scale.x = r_tag_pos_marker_.scale.y = r_tag_pos_marker_.scale.z = 0.15;
    r_tag_pos_marker_.text = "R";
    r_tag_pos_marker_.color.a = 1.0;
    r_tag_pos_marker_.color.r = 1.0;
    r_tag_pos_marker_.color.g = 1.0;
    obs_pos_marker_.ns = "observed_position";
    obs_pos_marker_.type = visualization_msgs::msg::Marker::SPHERE;
    obs_pos_marker_.scale.x = obs_pos_marker_.scale.y = obs_pos_marker_.scale.z = 0.308;
    obs_pos_marker_.color.a = 1.0;
    obs_pos_marker_.color.r = 1.0;
    pred_pos_marker_.ns = "predicted_position";
    pred_pos_marker_.type = visualization_msgs::msg::Marker::SPHERE;
    pred_pos_marker_.scale.x = pred_pos_marker_.scale.y = pred_pos_marker_.scale.z = 0.308;
    pred_pos_marker_.color.a = 1.0;
    pred_pos_marker_.color.g = 1.0;
    aimming_line_marker_.ns = "aimming_line";
    aimming_line_marker_.type = visualization_msgs::msg::Marker::ARROW;
    aimming_line_marker_.scale.x = 0.03;
    aimming_line_marker_.scale.y = 0.05;
    aimming_line_marker_.color.a = 0.5;
    aimming_line_marker_.color.r = 1.0;
    aimming_line_marker_.color.b = 1.0;
    aimming_line_marker_.color.g = 1.0;
    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "rune_solver/marker", rclcpp::SensorDataQoS());
  }
  last_rune_target_.header.frame_id = "";
  // Timer 250 Hz
  pub_timer_ = this->create_wall_timer(std::chrono::milliseconds(4),
                                       std::bind(&RuneSolverNode::timerCallback, this));

  // Heartbeat
  heartbeat_ = HeartBeatPublisher::create(this);
}

void RuneSolverNode::timerCallback() {
  // rune_solver_->pnp_solver is nullptr when camera_info is not received
  if (rune_solver_->pnp_solver == nullptr) {
    return;
  }

  // Return if not enable
  if (!enable_) {
    return;
  }

  // Init message
  rm_interfaces::msg::GimbalCmd control_msg;
  geometry_msgs::msg::PointStamped target_msg;
  target_msg.header.frame_id = "odom";

  // If target never detected
  if (last_rune_target_.header.frame_id.empty()) {
    control_msg.yaw_diff = 0;
    control_msg.pitch_diff = 0;
    control_msg.distance = -1;
    control_msg.pitch = 0;
    control_msg.yaw = 0;
    control_msg.fire_advice = false;
    gimbal_pub_->publish(control_msg);
    return;
  }

  double predict_angle = 0;

  // Calculate predict time
  Eigen::Vector3d cur_pos = rune_solver_->getTargetPosition(0);
  double flying_time = rune_solver_->trajectory_compensator->getFlyingTime(cur_pos);
  rclcpp::Time predict_timestamp = this->now() + rclcpp::Duration::from_seconds(predict_offset_) +
                                   rclcpp::Duration::from_seconds(flying_time);

  Eigen::Vector3d pred_pos = Eigen::Vector3d::Zero();

  if (rune_solver_->tracker_state == RuneSolver::TRACKING) {
    // Predict target
    predict_angle = rune_solver_->predictTarget(pred_pos, predict_timestamp.seconds());

    target_msg.header.stamp = predict_timestamp;
    target_msg.point.x = pred_pos.x();
    target_msg.point.y = pred_pos.y();
    target_msg.point.z = pred_pos.z();
    target_pub_->publish(target_msg);
    try {
      control_msg = rune_solver_->solveGimbalCmd(pred_pos);
    } catch (...) {
      FYT_ERROR("rune_solver", "solveGimbalCmd error");
      control_msg.yaw_diff = 0;
      control_msg.pitch_diff = 0;
      control_msg.distance = -1;
      control_msg.pitch = 0;
      control_msg.yaw = 0;
      control_msg.fire_advice = false;
    }
  } else {
    control_msg.yaw_diff = 0;
    control_msg.pitch_diff = 0;
    control_msg.distance = -1;
    control_msg.pitch = 0;
    control_msg.yaw = 0;
    control_msg.fire_advice = false;
  }
  gimbal_pub_->publish(control_msg);

  if (debug_) {
    // Publish fitting info
    std_msgs::msg::String fitter_text_msg;
    fitter_text_msg.data = fmt::format("{} | predict_time: {:.3f}s",
                                       rune_solver_->curve_fitter->getDebugText(),
                                       predict_offset_ + flying_time);
    fitter_text_pub_->publish(fitter_text_msg);

    // Publish visualization marker
    visualization_msgs::msg::MarkerArray marker_array;
    if (rune_solver_->tracker_state == RuneSolver::LOST) {
      obs_pos_marker_.action = visualization_msgs::msg::Marker::DELETEALL;
      pred_pos_marker_.action = visualization_msgs::msg::Marker::DELETEALL;
      r_tag_pos_marker_.action = visualization_msgs::msg::Marker::DELETEALL;
      aimming_line_marker_.action = visualization_msgs::msg::Marker::DELETEALL;
      marker_array.markers.push_back(obs_pos_marker_);
      marker_array.markers.push_back(pred_pos_marker_);
      marker_array.markers.push_back(r_tag_pos_marker_);
      marker_array.markers.push_back(aimming_line_marker_);
      marker_pub_->publish(marker_array);
    } else {
      obs_pos_marker_.header.frame_id = "odom";
      obs_pos_marker_.header.stamp = last_rune_target_.header.stamp;
      obs_pos_marker_.action = visualization_msgs::msg::Marker::ADD;
      obs_pos_marker_.lifetime = rclcpp::Duration::from_seconds(0.1);
      obs_pos_marker_.pose.position.x = cur_pos.x();
      obs_pos_marker_.pose.position.y = cur_pos.y();
      obs_pos_marker_.pose.position.z = cur_pos.z();

      Eigen::Vector3d r_tag_pos = rune_solver_->getCenterPosition();
      r_tag_pos_marker_.header.frame_id = "odom";
      r_tag_pos_marker_.header.stamp = last_rune_target_.header.stamp;
      r_tag_pos_marker_.action = visualization_msgs::msg::Marker::ADD;
      r_tag_pos_marker_.lifetime = rclcpp::Duration::from_seconds(0.1);
      r_tag_pos_marker_.pose.position.x = r_tag_pos.x();
      r_tag_pos_marker_.pose.position.y = r_tag_pos.y();
      r_tag_pos_marker_.pose.position.z = r_tag_pos.z();

      marker_array.markers.push_back(obs_pos_marker_);
      marker_array.markers.push_back(r_tag_pos_marker_);
      if (rune_solver_->tracker_state == RuneSolver::TRACKING) {
        pred_pos_marker_.header.frame_id = "odom";
        pred_pos_marker_.header.stamp = predict_timestamp;
        pred_pos_marker_.action = visualization_msgs::msg::Marker::ADD;
        pred_pos_marker_.lifetime = rclcpp::Duration::from_seconds(0.1);
        pred_pos_marker_.pose.position.x = pred_pos.x();
        pred_pos_marker_.pose.position.y = pred_pos.y();
        pred_pos_marker_.pose.position.z = pred_pos.z();
        marker_array.markers.push_back(pred_pos_marker_);

        aimming_line_marker_.action = visualization_msgs::msg::Marker::ADD;
        aimming_line_marker_.points.clear();
        aimming_line_marker_.header.frame_id = "odom";
        aimming_line_marker_.header.stamp = this->now();
        aimming_line_marker_.lifetime = rclcpp::Duration::from_seconds(0.1);
        geometry_msgs::msg::Point aimming_line_start, aimming_line_end;
        aimming_line_marker_.points.emplace_back(aimming_line_start);
        aimming_line_end.y = 15 * sin(control_msg.yaw * M_PI / 180);
        aimming_line_end.x = 15 * cos(control_msg.yaw * M_PI / 180);
        aimming_line_end.z = 15 * sin(control_msg.pitch * M_PI / 180);
        aimming_line_marker_.points.emplace_back(aimming_line_end);
        marker_array.markers.push_back(aimming_line_marker_);

        rm_interfaces::msg::DebugRuneAngle predict_angle_msg;
        predict_angle_msg.header = last_rune_target_.header;
        predict_angle_msg.header.stamp = predict_timestamp;
        predict_angle_msg.data = predict_angle;
        predicted_angle_pub_->publish(predict_angle_msg);
      }
      marker_pub_->publish(marker_array);
    }
  }
}

void RuneSolverNode::runeTargetCallback(
  const rm_interfaces::msg::RuneTarget::SharedPtr rune_target_msg) {
  // rune_solver_->pnp_solver is nullptr when camera_info is not received
  if (rune_solver_->pnp_solver == nullptr) {
    return;
  }

  // Keep the last detected target
  if (!rune_target_msg->is_lost) {
    last_rune_target_ = *rune_target_msg;
  }
  double observed_angle = 0;
  if (rune_solver_->tracker_state == RuneSolver::LOST) {
    observed_angle = rune_solver_->init(rune_target_msg);
  } else {
    observed_angle = rune_solver_->update(rune_target_msg);

    if (debug_) {
      rm_interfaces::msg::DebugRuneAngle observed_angle_msg;
      observed_angle_msg.header = rune_target_msg->header;
      observed_angle_msg.data = observed_angle;
      observed_angle_pub_->publish(observed_angle_msg);
    }
  }
}

rcl_interfaces::msg::SetParametersResult RuneSolverNode::onSetParameters(
  std::vector<rclcpp::Parameter> parameters) {
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  for (const auto &param : parameters) {
    if (param.get_name() == "predict_time") {
      predict_offset_ = param.as_double();
    } else if (param.get_name() == "debug") {
      debug_ = param.as_bool();
    } else if (param.get_name() == "gravity") {
      rune_solver_->rune_solver_params.gravity = param.as_double();
    } else if (param.get_name() == "bullet_speed") {
      rune_solver_->rune_solver_params.bullet_speed = param.as_double();
    } else if (param.get_name() == "angle_offset_thres") {
      rune_solver_->rune_solver_params.angle_offset_thres = param.as_double();
    } else if (param.get_name() == "lost_time_thres") {
      rune_solver_->rune_solver_params.lost_time_thres = param.as_double();
    }
  }
  return result;
}

void RuneSolverNode::setModeCallback(
  const std::shared_ptr<rm_interfaces::srv::SetMode::Request> request,
  std::shared_ptr<rm_interfaces::srv::SetMode::Response> response) {
  response->success = true;

  VisionMode mode = static_cast<VisionMode>(request->mode);
  std::string mode_name = visionModeToString(mode);
  if (mode_name == "UNKNOWN") {
    FYT_ERROR("rune_solver", "Invalid mode: {}", request->mode);
    return;
  }

  switch (mode) {
    case VisionMode::SMALL_RUNE_RED:
    case VisionMode::SMALL_RUNE_BLUE:
    case VisionMode::BIG_RUNE_RED:
    case VisionMode::BIG_RUNE_BLUE: {
      enable_ = true;
      break;
    }
    default: {
      enable_ = false;
      break;
    }
  }

  FYT_WARN("rune_solver", "Set Rune Mode: {}", visionModeToString(mode));
}

}  // namespace fyt::rune

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(fyt::rune::RuneSolverNode)
