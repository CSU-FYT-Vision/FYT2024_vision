// Created by Chengfu Zou
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

#include "rm_utils/math/trajectory_compensator.hpp"

namespace fyt {
bool TrajectoryCompensator::compensate(const Eigen::Vector3d &target_position,
                                       double &pitch) const noexcept {
  double target_height = target_position(2);
  // The iterative_height is used to calculate angle in each iteration
  double iterative_height = target_height;
  double impact_height = 0;
  double distance =
    std::sqrt(target_position(0) * target_position(0) + target_position(1) * target_position(1));
  double angle = std::atan2(target_height, distance);
  double dh = 0;
  // Iterate to find the right angle, which makes the impact height equal to the
  // target height
  for (int i = 0; i < iteration_times; ++i) {
    angle = std::atan2(iterative_height, distance);
    if (std::abs(angle) > M_PI / 2.5) {
      break;
    }
    impact_height = calculateTrajectory(distance, angle);
    dh = target_height - impact_height;
    if (std::abs(dh) < 0.01) {
      break;
    }
    iterative_height += dh;
  }
  if (std::abs(dh) > 0.01 || std::abs(angle) > M_PI / 2.5) {
    return false;
  }
  pitch = angle;
  return true;
}

std::vector<std::pair<double, double>> TrajectoryCompensator::getTrajectory(
  double distance, double angle) const noexcept {
  std::vector<std::pair<double, double>> trajectory;

  if (distance < 0) {
    return trajectory;
  }

  for (double x = 0; x < distance; x += 0.03) {
    trajectory.emplace_back(x, calculateTrajectory(x, angle));
  }
  return trajectory;
}

double IdealCompensator::calculateTrajectory(const double x, const double angle) const noexcept {
  double t = x / (velocity * cos(angle));
  double y = velocity * sin(angle) * t - 0.5 * gravity * t * t;
  return y;
}

double IdealCompensator::getFlyingTime(const Eigen::Vector3d &target_position) const noexcept {
  double distance =
    sqrt(target_position(0) * target_position(0) + target_position(1) * target_position(1));
  double angle = atan2(target_position(2), distance);
  double t = distance / (velocity * cos(angle));
  return t;
}

double ResistanceCompensator::calculateTrajectory(const double x,
                                                  const double angle) const noexcept {
  double r = resistance < 1e-4 ? 1e-4 : resistance;
  double t = (exp(r * x) - 1) / (r * velocity * cos(angle));
  double y = velocity * sin(angle) * t - 0.5 * gravity * t * t;
  return y;
}

double ResistanceCompensator::getFlyingTime(const Eigen::Vector3d &target_position) const noexcept {
  double r = resistance < 1e-4 ? 1e-4 : resistance;
  double distance =
    sqrt(target_position(0) * target_position(0) + target_position(1) * target_position(1));
  double angle = atan2(target_position(2), distance);
  double t = (exp(r * distance) - 1) / (r * velocity * cos(angle));
  return t;
}
}  // namespace fyt
