// Created by Chengfu Zou on 2024.1.19
// Copyright(C) FYT Vision Group. All rights resevred.
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

#include "rm_utils/math/pnp_solver.hpp"

#include <opencv2/calib3d.hpp>

namespace fyt {
PnPSolver::PnPSolver(const std::array<double, 9> &camera_matrix,
                     const std::vector<double> &distortion_coefficients,
                     cv::SolvePnPMethod method)
: camera_matrix_(cv::Mat(3, 3, CV_64F, const_cast<double *>(camera_matrix.data())).clone())
, distortion_coefficients_(
    cv::Mat(1, 5, CV_64F, const_cast<double *>(distortion_coefficients.data())).clone())
, method_(method) {}

void PnPSolver::setObjectPoints(const std::string &coord_frame_name,
                                const std::vector<cv::Point3f> &object_points) noexcept {
  object_points_map_[coord_frame_name] = object_points;
}

float PnPSolver::calculateDistanceToCenter(const cv::Point2f &image_point) const noexcept {
  float cx = camera_matrix_.at<double>(0, 2);
  float cy = camera_matrix_.at<double>(1, 2);
  return cv::norm(image_point - cv::Point2f(cx, cy));
}

Eigen::VectorXd PnPSolver::getPose(const cv::Mat &rvec, const cv::Mat &tvec) noexcept {
  auto pose = Eigen::VectorXd(6);
  cv::Mat rmat;
  cv::Rodrigues(rvec, rmat);

  float yaw, pitch, roll;
  yaw = std::asin(-rmat.at<double>(2, 0));
  pitch = std::atan2(rmat.at<double>(2, 1), rmat.at<double>(2, 2));
  roll = std::atan2(rmat.at<double>(1, 0), rmat.at<double>(0, 0));

  pose << tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2), yaw, pitch, -roll;
  return pose;
}

}  // namespace fyt
