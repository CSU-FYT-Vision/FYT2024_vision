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

#ifndef ARMOR_SOLVER_MOTION_MODEL_HPP_
#define ARMOR_SOLVER_MOTION_MODEL_HPP_

// ceres
#include <ceres/ceres.h>
// project
#include "rm_utils/math/extended_kalman_filter.hpp"

namespace fyt::auto_aim {

enum class MotionModel {
  CONSTANT_VELOCITY = 0,  // Constant velocity
  CONSTANT_ROTATION = 1,  // Constant rotation velocity
  CONSTANT_VEL_ROT = 2    // Constant velocity and rotation velocity
};

// X_N: state dimension, Z_N: measurement dimension
constexpr int X_N = 10, Z_N = 4;

struct Predict {
  explicit Predict(double dt, MotionModel model = MotionModel::CONSTANT_VEL_ROT)
  : dt(dt), model(model) {}

  template <typename T>
  void operator()(const T x0[X_N], T x1[X_N]) {
    for (int i = 0; i < X_N; i++) {
      x1[i] = x0[i];
    }

    // v_xyz
    if (model == MotionModel::CONSTANT_VEL_ROT || model == MotionModel::CONSTANT_VELOCITY) {
      // linear velocity
      x1[0] += x0[1] * dt;
      x1[2] += x0[3] * dt;
      x1[4] += x0[5] * dt;
    } else {
      // no velocity
      x1[1] *= 0.;
      x1[3] *= 0.;
      x1[5] *= 0.;
    }

    // v_yaw
    if (model == MotionModel::CONSTANT_VEL_ROT || model == MotionModel::CONSTANT_ROTATION) {
      // angular velocity
      x1[6] += x0[7] * dt;
    } else {
      // no rotation
      x1[7] *= 0.;
    }
  }

  double dt;
  MotionModel model;
};

struct Measure {
  template <typename T>
  void operator()(const T x[Z_N], T z[Z_N]) {
    z[0] = x[0] - ceres::cos(x[6]) * x[8];
    z[1] = x[2] - ceres::sin(x[6]) * x[8];
    z[2] = x[4] + x[9];
    z[3] = x[6];
  }
};

using RobotStateEKF = ExtendedKalmanFilter<X_N, Z_N, Predict, Measure>;

}  // namespace fyt::auto_aim
#endif
