// Copyright Chen Jun 2023. Licensed under the MIT License.
//
// Additional modifications and features by Chengfu Zou, Labor. Licensed under Apache License 2.0.
//
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


#include "rm_utils/math/extended_kalman_filter.hpp"

namespace fyt {
ExtendedKalmanFilter::ExtendedKalmanFilter(const VecVecFunc &f,
                                           const VecVecFunc &h,
                                           const VecMatFunc &j_f,
                                           const VecMatFunc &j_h,
                                           const VoidMatFunc &u_q,
                                           const VecMatFunc &u_r,
                                           const Eigen::MatrixXd &P0)
: f(f)
, h(h)
, jacobian_f(j_f)
, jacobian_h(j_h)
, update_Q(u_q)
, update_R(u_r)
, P_post(P0)
, n(P0.rows())
, I(Eigen::MatrixXd::Identity(n, n))
, x_pri(n)
, x_post(n) {}

void ExtendedKalmanFilter::setState(const Eigen::VectorXd &x0) noexcept { x_post = x0; }

Eigen::MatrixXd ExtendedKalmanFilter::predict() noexcept {
  F = jacobian_f(x_post), Q = update_Q();

  x_pri = f(x_post);
  P_pri = F * P_post * F.transpose() + Q;

  // handle the case when there will be no measurement before the next predict
  x_post = x_pri;
  P_post = P_pri;

  return x_pri;
}

Eigen::MatrixXd ExtendedKalmanFilter::update(const Eigen::VectorXd &z) noexcept {
  H = jacobian_h(x_pri), R = update_R(z);

  K = P_pri * H.transpose() * (H * P_pri * H.transpose() + R).inverse();
  x_post = x_pri + K * (z - h(x_pri));
  P_post = (I - K * H) * P_pri;

  return x_post;
}

}  // namespace fyt
