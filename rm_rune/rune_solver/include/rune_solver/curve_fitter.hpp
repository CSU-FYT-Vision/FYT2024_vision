// Created by Labor 2024.1.28
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

#ifndef RUNE_SOLVER_CURVE_FITTER_HPP_
#define RUNE_SOLVER_CURVE_FITTER_HPP_

// std
#include <future>
// third party
#include <ceres/ceres.h>
// project
#include "rune_solver/types.hpp"

namespace fyt::rune {

class CurveFitter {
public:
  explicit CurveFitter(const MotionType &t) : type_(t), fitting_future_(nullptr) {
    // Init parameters to be fitted
    fitting_param_ = {1.045, 0, 0, 0, 0};
    direction_ = Direction::UNKNOWN;
  };

  // Perform angle predict
  double predict(double time);

  // Update data to be fitted
  void update(double time, double angle);

  // Reset fitter
  void reset();

  // Check the state of fitter
  bool statusVerified();

  // Set the type of fitter
  void setType(const MotionType &t);

  void setAutoTypeDetermined(bool auto_type_determined);

  MotionType getType() const;

  // Get the string of the fitting result
  std::string getDebugText();

private:
  // Perform double curve fitting
  // automated determination of the type of curve
  void fitDoubleCurve();

  // Perform curve fitting
  void fitCurve();

  // Status value
  MotionType type_;
  bool is_static_ = false;
  bool auto_type_determined_ = false;
  int direction_;

  // Data to be fitted
  static constexpr int QUEUE_UPPER_LIMIT = 500;
  static constexpr int QUEUE_LOWER_LIMIT = 50;
  struct Data {
    double time;
    double angle;
  };
  std::deque<Data> data_history_queue_;

  // Parameters to be fitted
  std::array<double, 5> fitting_param_;
  std::unique_ptr<std::future<void>> fitting_future_;

private:
  // Fitting Curve
#define BIG_RUNE_CURVE(x, a, omega, b, c, d, sign) \
  ((-((a) / (omega) * ceres::cos((omega) * ((x) + (d)))) + (b) * ((x) + (d)) + (c)) * (sign))

#define SMALL_RUNE_CURVE(x, a, b, c, sign) (((a) * ((x) + (b)) + (c)) * (sign))

  // Fitting Cost
  struct BigRuneFittingCost {
    BigRuneFittingCost(double x, double y, int m) : x_(x), y_(y), mov_(static_cast<double>(m)) {}
    template <typename T>
    bool operator()(const T *const p, T *residual) const {
      residual[0] = y_ - BIG_RUNE_CURVE(x_, p[0], p[1], p[2], p[3], p[4], mov_);
      return true;
    }

    const double x_, y_;
    const double mov_;
  };

  struct SmallRuneFittingCost {
    SmallRuneFittingCost(double x, double y, int m) : x_(x), y_(y), mov_(static_cast<double>(m)) {}
    template <typename T>
    bool operator()(const T *const p, T *residual) const {
      residual[0] = y_ - SMALL_RUNE_CURVE(x_, p[0], p[1], p[2], mov_);
      return true;
    }

    const double x_, y_;
    const double mov_;
  };
};
}  //namespace fyt::rune
#endif  // RUNE_SOLVER_CURVE_FITTER_HPP_
