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

#ifndef RUNE_SOLVER_EKF_FUNCTIONS_HPP_
#define RUNE_SOLVER_EKF_FUNCTIONS_HPP_

#include <ceres/ceres.h>

#include "rm_utils/math/extended_kalman_filter.hpp"

namespace fyt::rune {

constexpr int X_N = 4, Z_N = 4;

struct Predict {
  template <typename T>
  void operator()(const T x0[X_N], T x1[X_N]) {
    for (int i = 0; i < X_N; ++i) {
      x1[i] = x0[i];
    }
  }
};

struct Measure {
  template <typename T>
  void operator()(const T x[Z_N], T z[Z_N]) {
    for (int i = 0; i < Z_N; ++i) {
      z[i] = x[i];
    }
  }
};

using RuneCenterEKF = ExtendedKalmanFilter<X_N, Z_N, Predict, Measure>;

}  // namespace fyt::rune
#endif
