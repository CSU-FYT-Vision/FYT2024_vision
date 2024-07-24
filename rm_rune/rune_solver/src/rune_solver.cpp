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

#include "rune_solver/rune_solver.hpp"

// ros2
#include <cv_bridge/cv_bridge.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
// std
#include <memory>
// third party
#include <angles/angles.h>

#include <Eigen/Geometry>
// project
#include "rm_utils/logger/log.hpp"
#include "rm_utils/math/utils.hpp"
#include "rune_solver/types.hpp"

namespace fyt::rune {
RuneSolver::RuneSolver(const RuneSolverParams &rsp, std::shared_ptr<tf2_ros::Buffer> buffer)
: rune_solver_params(rsp), tf2_buffer_(buffer) {
  // Init
  tracker_state = LOST;
  curve_fitter = std::make_unique<CurveFitter>(MotionType::UNKNOWN);
  curve_fitter->setAutoTypeDetermined(rsp.auto_type_determined);
  trajectory_compensator = CompensatorFactory::createCompensator(rsp.compensator_type);
  trajectory_compensator->gravity = rsp.gravity;
  trajectory_compensator->velocity = rsp.bullet_speed;
  trajectory_compensator->resistance = 0.01;
  ekf_state_ = Eigen::Vector4d::Zero();
  manual_compensator = std::make_unique<ManualCompensator>();
}

double RuneSolver::init(const rm_interfaces::msg::RuneTarget::SharedPtr received_target) {
  if (received_target->is_lost) {
    return 0;
  }

  FYT_INFO("rune_solver", "Init!");

  // Init EKF
  try {
    Eigen::Matrix4d T_odom_2_rune = solvePose(*received_target);

    // Filter out outliers
    Eigen::Vector3d t = T_odom_2_rune.block(0, 3, 3, 1);
    if (t.norm() < MIN_RUNE_DISTANCE || t.norm() > MAX_RUNE_DISTANCE) {
      FYT_ERROR("rune_solver", "Rune position is out of range");
      return 0;
    }

    ekf_state_ = getStateFromTransform(T_odom_2_rune);
    ekf->setState(ekf_state_);
  } catch (...) {
    FYT_ERROR("rune_solver", "Init failed");
    return 0;
  }

  // Init observation variables
  tracker_state = DETECTING;
  double observed_angle = getNormalAngle(received_target);
  double observed_time = 0;
  curve_fitter->update(observed_time, observed_angle);

  last_observed_angle_ = observed_angle;
  last_angle_ = last_observed_angle_;
  start_time_ = rclcpp::Time(received_target->header.stamp).seconds();
  last_time_ = start_time_;

  return observed_angle;
}

double RuneSolver::update(const rm_interfaces::msg::RuneTarget::SharedPtr received_target) {
  double now_time = rclcpp::Time(received_target->header.stamp).seconds();
  double delta_time = now_time - last_time_;

  if (received_target->is_big_rune) {
    curve_fitter->setType(MotionType::BIG);
  } else {
    curve_fitter->setType(MotionType::SMALL);
  }

  if (!received_target->is_lost) {
    // Update EKF
    try {
      Eigen::Matrix4d T_odom_2_rune = solvePose(*received_target);

      // Filter out outliers
      Eigen::Vector3d t = T_odom_2_rune.block(0, 3, 3, 1);
      if (t.norm() < MIN_RUNE_DISTANCE || t.norm() > MAX_RUNE_DISTANCE) {
        FYT_ERROR("rune_solver", "Rune position is out of range");
        return 0;
      }

      Eigen::Vector4d measurement = getStateFromTransform(T_odom_2_rune);
      ekf->predict();
      ekf_state_ = ekf->update(measurement);
    } catch (...) {
      FYT_ERROR("rune_solver", "EKF update failed");
      return 0;
    }

    // Get the data to be fitted
    double observed_time = now_time - start_time_;
    double normal_angle = getNormalAngle(received_target);
    double observed_angle = getObservedAngle(normal_angle);

    // Update fitter
    curve_fitter->update(observed_time, observed_angle);

    last_time_ = now_time;
    last_angle_ = normal_angle;
    last_observed_angle_ = observed_angle;
  }

  // Update tracker state
  switch (tracker_state) {
    case DETECTING: {
      if (received_target->is_lost && delta_time > rune_solver_params.lost_time_thres) {
        tracker_state = LOST;
        curve_fitter->reset();
      } else if (curve_fitter->statusVerified()) {
        tracker_state = TRACKING;
      }
      break;
    }
    case TRACKING: {
      if (received_target->is_lost && delta_time > rune_solver_params.lost_time_thres) {
        tracker_state = LOST;
        curve_fitter->reset();
      }
      break;
    }
    case LOST: {
      if (!received_target->is_lost) {
        tracker_state = DETECTING;
      }
      break;
    }
  }
  return last_observed_angle_;
}

double RuneSolver::predictTarget(Eigen::Vector3d &predicted_position, double timestamp) {
  double t1 = timestamp - start_time_;
  double t0 = last_time_ - start_time_;
  double predict_angle_diff = curve_fitter->predict(t1) - curve_fitter->predict(t0);

  // Get the predicted position
  predicted_position = getTargetPosition(predict_angle_diff);

  return predict_angle_diff + last_observed_angle_;
}

Eigen::Matrix4d RuneSolver::solvePose(const rm_interfaces::msg::RuneTarget &predicted_target) {
  Eigen::Matrix4d pose = Eigen::Matrix4d::Identity();
  std::vector<cv::Point2f> image_points(predicted_target.pts.size());
  std::transform(predicted_target.pts.begin(),
                 predicted_target.pts.end(),
                 image_points.begin(),
                 [](const auto &pt) { return cv::Point2f(pt.x, pt.y); });

  cv::Mat rvec(3, 1, CV_64F), tvec(3, 1, CV_64F);
  if (pnp_solver && pnp_solver->solvePnP(image_points, rvec, tvec, "rune")) {
    // Get the transformation matrix from rune to odom
    try {
      // Get rotation matrix from rvec
      cv::Mat rmat;
      cv::Rodrigues(rvec, rmat);
      Eigen::Matrix3d rot;
      // clang-format off
      rot << rmat.at<double>(0, 0), rmat.at<double>(0, 1), rmat.at<double>(0, 2),
             rmat.at<double>(1, 0), rmat.at<double>(1, 1), rmat.at<double>(1, 2), 
             rmat.at<double>(2, 0), rmat.at<double>(2, 1), rmat.at<double>(2, 2);
      // clang-format on
      Eigen::Quaterniond quat(rot);

      // Init pose msg
      geometry_msgs::msg::PoseStamped ps;
      ps.header.frame_id = "camera_optical_frame";
      ps.header.stamp = predicted_target.header.stamp;

      // Fill pose msg
      ps.pose.orientation.x = quat.x();
      ps.pose.orientation.y = quat.y();
      ps.pose.orientation.z = quat.z();
      ps.pose.orientation.w = quat.w();
      ps.pose.position.x = tvec.at<double>(0);
      ps.pose.position.y = tvec.at<double>(1);
      ps.pose.position.z = tvec.at<double>(2);

      // Transform to odom
      ps = tf2_buffer_->transform(ps, "odom");

      // Fill pose
      pose(0, 3) = ps.pose.position.x;
      pose(1, 3) = ps.pose.position.y;
      pose(2, 3) = ps.pose.position.z;

      Eigen::Quaterniond quat_odom;
      quat_odom.x() = ps.pose.orientation.x;
      quat_odom.y() = ps.pose.orientation.y;
      quat_odom.z() = ps.pose.orientation.z;
      quat_odom.w() = ps.pose.orientation.w;

      Eigen::Matrix3d rot_odom = quat_odom.toRotationMatrix();
      pose.block(0, 0, 3, 3) = rot_odom;

    } catch (tf2::TransformException &ex) {
      FYT_ERROR("rune_solver", "rune to odom error: {}", ex.what());
      throw ex;
    }
  } else {
    FYT_ERROR("rune_solver", "PnP failed");
    throw std::runtime_error("PnP failed");
  }
  return pose;
}

rm_interfaces::msg::GimbalCmd RuneSolver::solveGimbalCmd(const Eigen::Vector3d &target) {
  // Get current yaw and pitch of gimbal
  double current_yaw = 0.0, current_pitch = 0.0;
  try {
    auto gimbal_tf = tf2_buffer_->lookupTransform("odom", "gimbal_link", tf2::TimePointZero);
    auto msg_q = gimbal_tf.transform.rotation;

    tf2::Quaternion tf_q;
    tf2::fromMsg(msg_q, tf_q);
    double roll;
    tf2::Matrix3x3(tf_q).getRPY(roll, current_pitch, current_yaw);
    current_pitch = -current_pitch;
  } catch (tf2::TransformException &ex) {
    FYT_ERROR("rune_solver", "{}", ex.what());
    throw ex;
  }

  // Calculate yaw and pitch
  double yaw = atan2(target.y(), target.x());
  double pitch = atan2(target.z(), target.head(2).norm());

  // Set parameters of compensator
  trajectory_compensator->velocity = rune_solver_params.bullet_speed;
  trajectory_compensator->gravity = rune_solver_params.gravity;
  trajectory_compensator->iteration_times = 30;

  if (double temp_pitch = pitch; trajectory_compensator->compensate(target, temp_pitch)) {
    pitch = temp_pitch;
  }
  double distance = target.norm();

  // Compensate angle by angle_offset_map
  auto angle_offset = manual_compensator->angleHardCorrect(target.head(2).norm(), target.z());
  double pitch_offset = angle_offset[0] * M_PI / 180;
  double yaw_offset = angle_offset[1] * M_PI / 180;
  double cmd_pitch = pitch + pitch_offset;
  double cmd_yaw = angles::normalize_angle(yaw + yaw_offset);

  rm_interfaces::msg::GimbalCmd gimbal_cmd;
  gimbal_cmd.yaw = cmd_yaw * 180 / M_PI;
  gimbal_cmd.pitch = cmd_pitch * 180 / M_PI;
  gimbal_cmd.yaw_diff = (cmd_yaw - current_yaw) * 180 / M_PI;
  gimbal_cmd.pitch_diff = (cmd_pitch - current_pitch) * 180 / M_PI;
  gimbal_cmd.distance = distance;

  // Judge whether to shoot
  constexpr double TARGET_RADIUS = 0.308;
  double shooting_range_yaw = std::abs(atan2(TARGET_RADIUS / 2, distance)) * 180 / M_PI;
  double shooting_range_pitch = std::abs(atan2(TARGET_RADIUS / 2, distance)) * 180 / M_PI;
  // Limit the shooting area to 1 degree to avoid not shooting when distance is
  // too large
  shooting_range_yaw = std::max(shooting_range_yaw, 1.0);
  shooting_range_pitch = std::max(shooting_range_pitch, 1.0);
  if (std::abs(gimbal_cmd.yaw_diff) < shooting_range_yaw &&
      std::abs(gimbal_cmd.pitch_diff) < shooting_range_pitch) {
    gimbal_cmd.fire_advice = true;
    FYT_DEBUG("rune_solver", "You Can Fire!");
  } else {
    gimbal_cmd.fire_advice = false;
  }

  return gimbal_cmd;
}

double RuneSolver::getNormalAngle(const rm_interfaces::msg::RuneTarget::SharedPtr received_target) {
  auto center_point = cv::Point2f(received_target->pts[0].x, received_target->pts[0].y);
  std::array<cv::Point2f, ARMOR_KEYPOINTS_NUM> armor_points;
  std::transform(received_target->pts.begin() + 1,
                 received_target->pts.end(),
                 armor_points.begin(),
                 [](const auto &pt) { return cv::Point2f(pt.x, pt.y); });

  cv::Point2f armor_center = getCenterPoint(armor_points);
  double x_diff = armor_center.x - center_point.x;
  double y_diff = -(armor_center.y - center_point.y);
  double normal_angle = std::atan2(y_diff, x_diff);
  // Normalize angle
  normal_angle = angles::normalize_angle_positive(normal_angle);

  return normal_angle;
}

double RuneSolver::getObservedAngle(double normal_angle) {
  double angle_diff = angles::shortest_angular_distance(last_angle_, normal_angle);
  // Handle rune target switch
  if (std::abs(angle_diff) > rune_solver_params.angle_offset_thres) {
    angle_diff = normal_angle - last_angle_;
    int offset = std::round(double(angle_diff / DEG_72));
    angle_diff -= offset * DEG_72;
  }

  double observed_angle = last_observed_angle_ + angle_diff;

  return observed_angle;
}

Eigen::Vector3d RuneSolver::getCenterPosition() const { return ekf_state_.head(3); }

Eigen::Vector3d RuneSolver::getTargetPosition(double angle_diff) const {
  Eigen::Vector3d t_odom_2_rune = ekf_state_.head(3);

  // Considering the large error and jitter(抖动) in the orientation obtained from PnP,
  // and the fact that the position of the Rune are static in the odom frame,
  // it is advisable to reconstruct the rotation matrix using geometric information
  double yaw = ekf_state_(3);
  double pitch = 0;
  double roll = -last_angle_;
  Eigen::Matrix3d R_odom_2_rune =
    utils::eulerToMatrix(Eigen::Vector3d{roll, pitch, yaw}, utils::EulerOrder::XYZ);

  // Calculate the position of the armor in rune frame
  Eigen::Vector3d p_rune = Eigen::AngleAxisd(-angle_diff, Eigen::Vector3d::UnitX()).matrix() *
                           Eigen::Vector3d(0, -ARM_LENGTH, 0);

  // Transform to odom frame
  Eigen::Vector3d p_odom = R_odom_2_rune * p_rune + t_odom_2_rune;

  return p_odom;
}

Eigen::Vector4d RuneSolver::getStateFromTransform(const Eigen::Matrix4d &transform) const {
  // Get yaw
  Eigen::Matrix3d R_odom_2_rune = transform.block(0, 0, 3, 3);
  Eigen::Quaterniond q_eigen = Eigen::Quaterniond(R_odom_2_rune);
  tf2::Quaternion q_tf = tf2::Quaternion(q_eigen.x(), q_eigen.y(), q_eigen.z(), q_eigen.w());
  double roll, pitch, yaw;
  tf2::Matrix3x3(q_tf).getRPY(roll, pitch, yaw);
  yaw = angles::normalize_angle(yaw);

  // Make yaw continuos
  yaw = ekf_state_(3) + angles::shortest_angular_distance(ekf_state_(3), yaw);

  Eigen::Vector4d state;
  state << transform(0, 3), transform(1, 3), transform(2, 3), yaw;
  return state;
}

double RuneSolver::getCurAngle() const { return last_angle_; }

cv::Point2f RuneSolver::getCenterPoint(
  const std::array<cv::Point2f, ARMOR_KEYPOINTS_NUM> &armor_points) {
  return std::accumulate(armor_points.begin(), armor_points.end(), cv::Point2f(0, 0)) /
         ARMOR_KEYPOINTS_NUM;
}

}  // namespace fyt::rune
