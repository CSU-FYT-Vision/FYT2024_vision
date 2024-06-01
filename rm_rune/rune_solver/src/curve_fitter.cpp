// Created by Labor 2024.1.28
// Maintained by Chengfu Zou
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

#include "rune_solver/curve_fitter.hpp"
// std
#include <cmath>
#include <string>
#include <thread>
// third party
#include <fmt/format.h>
// project
#include "rm_utils/logger/log.hpp"
#include "rune_solver/types.hpp"

namespace fyt::rune {

// Fit two curves simultaneously and choose the one with lower cost
// This function will change the type_ automatically
void CurveFitter::fitDoubleCurve() {
  if (is_static_) {
    // Treat the static target as a small rune
    type_ = MotionType::SMALL;
    return;
  }

  constexpr int PARALLEL_THRESHOLD = 300;
  bool is_parallel = data_history_queue_.size() > PARALLEL_THRESHOLD;

  auto t1 = std::chrono::high_resolution_clock::now();

  ceres::Problem small_fitting_problem;
  ceres::Problem big_fitting_problem;

  std::array<double, 5> small_param;
  std::array<double, 5> big_param;

  // Set the initial parameters in different cases
  switch (type_) {
    case MotionType::UNKNOWN: {
      small_param = {1.045, 0, 0, 0, 0};
      big_param = {0.9125, 1.942, 2.090 - 0.9125, 0, 0};
      break;
    }
    case MotionType::BIG: {
      small_param = {1.045, 0, 0, 0, 0};
      big_param = fitting_param_;
      break;
    }
    case MotionType::SMALL: {
      small_param = fitting_param_;
      big_param = {0.9125, 1.942, 2.090 - 0.9125, 0, 0};
      break;
    }
  }

  // Add residuals to the problems
  double *big_param_ptr = big_param.data();
  double *small_param_ptr = small_param.data();
  std::for_each(data_history_queue_.begin(), data_history_queue_.end(), [&](const auto &data) {
    small_fitting_problem.AddResidualBlock(
      new ceres::AutoDiffCostFunction<CurveFitter::SmallRuneFittingCost, 1, 3>(
        new SmallRuneFittingCost(data.time, data.angle, direction_)),
      new ceres::CauchyLoss(0.5),
      small_param_ptr);

    big_fitting_problem.AddResidualBlock(
      new ceres::AutoDiffCostFunction<CurveFitter::BigRuneFittingCost, 1, 5>(
        new BigRuneFittingCost(data.time, data.angle, direction_)),
      new ceres::CauchyLoss(0.5),
      big_param_ptr);
  });

  // Set the bounds of the parameters
  big_fitting_problem.SetParameterLowerBound(big_param_ptr, 0, 0.780 * 0.5);
  big_fitting_problem.SetParameterUpperBound(big_param_ptr, 0, 1.045 * 1.5);
  big_fitting_problem.SetParameterLowerBound(big_param_ptr, 1, 1.884 * 0.5);
  big_fitting_problem.SetParameterUpperBound(big_param_ptr, 1, 2.000 * 1.5);
  big_fitting_problem.SetParameterLowerBound(big_param_ptr, 2, (2.090 - 1.045) * 0.5);
  big_fitting_problem.SetParameterUpperBound(big_param_ptr, 2, (2.090 - 0.780) * 1.5);
  small_fitting_problem.SetParameterLowerBound(small_param_ptr, 0, 1.045 * 0.5);
  small_fitting_problem.SetParameterUpperBound(small_param_ptr, 0, 1.045 * 1.5);

  ceres::Solver::Options options;
  options.linear_solver_type = ceres::DENSE_QR;
  ceres::Solver::Summary small_summary;
  ceres::Solver::Summary big_summary;

  // Start the optimization
  if (is_parallel) {
    auto f1 = std::async(std::launch::async,
                         [&]() { ceres::Solve(options, &small_fitting_problem, &small_summary); });
    auto f2 = std::async(std::launch::async,
                         [&]() { ceres::Solve(options, &big_fitting_problem, &big_summary); });
    f1.get();
    f2.get();
  } else {
    ceres::Solve(options, &small_fitting_problem, &small_summary);
    ceres::Solve(options, &big_fitting_problem, &big_summary);
  }

  double small_cost = small_summary.final_cost;
  double big_cost = big_summary.final_cost;
  // Choose the curve with lower cost
  if (small_cost < big_cost) {
    fitting_param_ = small_param;
    type_ = MotionType::SMALL;
  } else {
    fitting_param_ = big_param;
    type_ = MotionType::BIG;
  }
  auto t2 = std::chrono::high_resolution_clock::now();
  FYT_DEBUG("rune_solver",
            "Fitting time: {} ms",
            std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count());
}

// Fit the curve with the determined type
void CurveFitter::fitCurve() {
  if (is_static_) {
    return;
  }

  auto t1 = std::chrono::high_resolution_clock::now();
  ceres::Problem problem;

  // Copy the fitting parameters to a temporary array
  // to avoid using the variables that are being optimized
  std::array<double, 5> temp_param = fitting_param_;

  // If the target is static, set the initial parameters, optimize the curve from scratch
  if (type_ == MotionType::BIG) {
    temp_param = {0.9125, 1.942, 2.090 - 0.9125, 0, 0};
  } else if (type_ == MotionType::SMALL) {
    temp_param = {1.045, 0, 0, 0, 0};
  }

  double *param_ptr = temp_param.data();

  // Add residuals to the problem
  std::for_each(data_history_queue_.begin(), data_history_queue_.end(), [&](const auto &data) {
    if (type_ == MotionType::BIG) {
      problem.AddResidualBlock(
        new ceres::AutoDiffCostFunction<CurveFitter::BigRuneFittingCost, 1, 5>(
          new BigRuneFittingCost(data.time, data.angle, direction_)),
        new ceres::CauchyLoss(0.5),
        param_ptr);
    } else if (type_ == MotionType::SMALL) {
      problem.AddResidualBlock(
        new ceres::AutoDiffCostFunction<CurveFitter::SmallRuneFittingCost, 1, 3>(
          new SmallRuneFittingCost(data.time, data.angle, direction_)),
        new ceres::CauchyLoss(0.5),
        param_ptr);
    }
  });

  // Set the bounds of the parameters
  if (type_ == MotionType::BIG) {
    problem.SetParameterLowerBound(param_ptr, 0, 0.780 * 0.5);
    problem.SetParameterUpperBound(param_ptr, 0, 1.045 * 1.5);
    problem.SetParameterLowerBound(param_ptr, 1, 1.884 * 0.5);
    problem.SetParameterUpperBound(param_ptr, 1, 2.000 * 1.5);
    problem.SetParameterLowerBound(param_ptr, 2, (2.090 - 1.045) * 0.5);
    problem.SetParameterUpperBound(param_ptr, 2, (2.090 - 0.780) * 1.5);
  } else if (type_ == MotionType::SMALL) {
    problem.SetParameterLowerBound(param_ptr, 0, 1.045 * 0.5);
    problem.SetParameterUpperBound(param_ptr, 0, 1.045 * 1.5);
  }

  // Start the optimization
  ceres::Solver::Options options;
  options.linear_solver_type = ceres::DENSE_QR;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  auto t2 = std::chrono::high_resolution_clock::now();
  FYT_DEBUG("rune_solver",
            "Fitting time: {} ms",
            std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count());

  // Save the optimized parameters
  fitting_param_ = temp_param;
}

double CurveFitter::predict(double current_time) {
  // If the target is static, return the last angle
  if (is_static_) {
    return data_history_queue_.back().angle;
  }

  double pred_angle = 0;
  if (type_ == MotionType::BIG) {
    pred_angle = BIG_RUNE_CURVE(current_time,
                                fitting_param_[0],
                                fitting_param_[1],
                                fitting_param_[2],
                                fitting_param_[3],
                                fitting_param_[4],
                                direction_);
  } else if (type_ == MotionType::SMALL) {
    pred_angle = SMALL_RUNE_CURVE(
      current_time, fitting_param_[0], fitting_param_[1], fitting_param_[2], direction_);
  }

  return pred_angle;
}

void CurveFitter::reset() {
  if (fitting_future_ != nullptr && fitting_future_->valid()) {
    fitting_future_->wait();
    fitting_future_.reset();
  }
  type_ = MotionType::UNKNOWN;
  direction_ = Direction::UNKNOWN;
  data_history_queue_.clear();
}

void CurveFitter::setType(const MotionType &t) {
  // Only available when the auto_type_determined_ is false
  if (type_ == t || auto_type_determined_) {
    return;
  }

  if (fitting_future_ != nullptr && fitting_future_->valid()) {
    fitting_future_->wait();
  }

  type_ = t;
  if (t == MotionType::BIG) {
    fitting_param_ = {0.9125, 1.942, 2.090 - 0.9125, 0, 0};
  } else if (t == MotionType::SMALL) {
    fitting_param_ = {1.045, 0, 0, 0, 0};
  }
}

MotionType CurveFitter::getType() const { return type_; }

void CurveFitter::setAutoTypeDetermined(bool auto_type_determined) {
  auto_type_determined_ = auto_type_determined;
}

std::string CurveFitter::getDebugText() {
  std::string t = "Unknown";

  if (type_ == MotionType::BIG) {
    double a = fitting_param_[0];
    double omega = fitting_param_[1];
    double b = fitting_param_[2];
    // double c = fitting_param_[3];
    double d = fitting_param_[4];
    t = fmt::format("V: {}( {:.2f} sin( {:.2f} (x {} {:.2f}) ) {} {:.2f} )",
                    direction_ == Direction::CLOCKWISE ? "-" : " ",
                    a,
                    omega,
                    (d > 0 ? "+" : "-"),
                    std::abs(d),
                    (b > 0 ? "+" : "-"),
                    std::abs(b));

  } else if (type_ == MotionType::SMALL) {
    double v = is_static_ ? 0 : fitting_param_[0];
    t = fmt::format("V: {:.2f}", direction_ * v);
  }
  return t;
}

void CurveFitter::update(double time, double angle) {
  data_history_queue_.emplace_back(Data{.time = time, .angle = angle});

  // Start fitting when the queue size reaches the lower limit
  if (data_history_queue_.size() < QUEUE_LOWER_LIMIT) {
    return;
  }

  // Limit the size of the queue
  if (data_history_queue_.size() > QUEUE_UPPER_LIMIT) {
    data_history_queue_.pop_front();
  }

  // Check if the target is moving or static
  double angle_diff = data_history_queue_.back().angle - data_history_queue_.front().angle;
  if (std::abs(angle_diff) < 2 * CV_PI / 180) {
    is_static_ = true;
    data_history_queue_.pop_front();
  } else {
    is_static_ = false;
  }

  // Determine the direction of rotation
  direction_ = static_cast<int>(angle_diff < 0 ? Direction::CLOCKWISE : Direction::ANTI_CLOCKWISE);

  auto startFitting = [this]() {
    if (auto_type_determined_) {
      fitting_future_ = std::make_unique<std::future<void>>(
        std::async(std::launch::async, &CurveFitter::fitDoubleCurve, this));
    } else {
      fitting_future_ = std::make_unique<std::future<void>>(
        std::async(std::launch::async, &CurveFitter::fitCurve, this));
    }
  };

  if (fitting_future_ == nullptr) {
    // First fitting
    startFitting();
    // Wait for the first fitting to finish in case the fitting is not finished when the first prediction is needed
    fitting_future_->wait();
  } else if (fitting_future_->wait_for(std::chrono::seconds(0)) == std::future_status::ready) {
    // Start a new fitting asynchronously
    startFitting();
  } else {
    // If fitting is in progess or has been completed, do not start a new fitting
    FYT_WARN("rune_solver", "Fitting is in progress, do not start a new fitting");
  }
}

bool CurveFitter::statusVerified() {
  if (type_ == MotionType::UNKNOWN || direction_ == Direction::UNKNOWN ||
      fitting_future_ == nullptr) {
    return false;
  }
  return true;
}
}  //namespace fyt::rune
