// Copyright Chen Jun 2023. Licensed under the MIT License.
// Copyright xinyang 2021.
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

#ifndef RM_UTILS_KALMAN_FILTER_HPP_
#define RM_UTILS_KALMAN_FILTER_HPP_

// std
#include <functional>
// Eigen
#include <Eigen/Dense>
// ceres
#include <ceres/jet.h>

namespace fyt {

// Extended Kalman Filter with auto differentiation
// N_X: state vector dimension
// N_Z: measurement vector dimension
// PredicFunc: process nonlinear vector function
// MeasureFunc: observation nonlinear vector function
template <int N_X, int N_Z, class PredicFunc, class MeasureFunc>
class ExtendedKalmanFilter {
public:
  ExtendedKalmanFilter() = default;

  using MatrixXX = Eigen::Matrix<double, N_X, N_X>;
  using MatrixZX = Eigen::Matrix<double, N_Z, N_X>;
  using MatrixXZ = Eigen::Matrix<double, N_X, N_Z>;
  using MatrixZZ = Eigen::Matrix<double, N_Z, N_Z>;
  using MatrixX1 = Eigen::Matrix<double, N_X, 1>;
  using MatrixZ1 = Eigen::Matrix<double, N_Z, 1>;

  using UpdateQFunc = std::function<MatrixXX()>;
  using UpdateRFunc = std::function<MatrixZZ(const MatrixZ1 &z)>;

  explicit ExtendedKalmanFilter(const PredicFunc &f,
                                const MeasureFunc &h,
                                const UpdateQFunc &u_q,
                                const UpdateRFunc &u_r,
                                const MatrixXX &P0) noexcept
  : f(f), h(h), update_Q(u_q), update_R(u_r), P_post(P0) {
    F = MatrixXX::Zero();
    H = MatrixZX::Zero();
  }

  // Set the initial state
  void setState(const MatrixX1 &x0) noexcept { x_post = x0; }

  void setPredictFunc(const PredicFunc &f) noexcept { this->f = f; }

  void setMeasureFunc(const MeasureFunc &h) noexcept { this->h = h; }

  // Compute a predicted state
  MatrixX1 predict() noexcept {
    ceres::Jet<double, N_X> x_e_jet[N_X];
    for (int i = 0; i < N_X; ++i) {
      x_e_jet[i].a = x_post[i];
      x_e_jet[i].v[i] = 1.;
      // a 对自己的偏导数为 1.
    }
    ceres::Jet<double, N_X> x_p_jet[N_X];
    f(x_e_jet, x_p_jet);

    for (int i = 0; i < N_X; ++i) {
      x_pri[i] = x_p_jet[i].a;
      F.block(i, 0, 1, N_X) = x_p_jet[i].v.transpose();
    }

    Q = update_Q();
    P_pri = F * P_post * F.transpose() + Q;
    x_post = x_pri;

    return x_pri;
  }

  // Update the estimated state based on measurement
  MatrixX1 update(const MatrixZ1 &z) noexcept {
    ceres::Jet<double, N_X> x_p_jet[N_X];
    for (int i = 0; i < N_X; i++) {
      x_p_jet[i].a = x_pri[i];
      x_p_jet[i].v[i] = 1;
    }
    ceres::Jet<double, N_X> z_p_jet[N_Z];
    h(x_p_jet, z_p_jet);

    MatrixZ1 z_pri;
    for (int i = 0; i < N_Z; i++) {
      z_pri[i] = z_p_jet[i].a;
      H.block(i, 0, 1, N_X) = z_p_jet[i].v.transpose();
    }

    R = update_R(z);
    K = P_pri * H.transpose() * (H * P_pri * H.transpose() + R).inverse();
    x_post = x_post + K * (z - z_pri);
    P_post = (MatrixXX::Identity() - K * H) * P_pri;
    return x_post;
  }

private:
  // Process nonlinear vector function
  PredicFunc f;
  MatrixXX F;
  // Observation nonlinear vector function
  MeasureFunc h;
  MatrixZX H;
  // Process noise covariance matrix
  UpdateQFunc update_Q;
  MatrixXX Q;
  // Measurement noise covariance matrix
  UpdateRFunc update_R;
  MatrixZZ R;

  // Priori error estimate covariance matrix
  MatrixXX P_pri;
  // Posteriori error estimate covariance matrix
  MatrixXX P_post;

  // Kalman gain
  MatrixXZ K;

  // Priori state
  MatrixX1 x_pri;
  // Posteriori state
  MatrixX1 x_post;
};

}  // namespace fyt

#endif  // RM_UTILS_KALMAN_FILTER_HPP_
