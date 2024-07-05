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

#ifndef RM_UTILS_MATH_PARTICLE_FILTER_HPP_
#define RM_UTILS_MATH_PARTICLE_FILTER_HPP_

#include <Eigen/Dense>
#include <random>
#include <set>

namespace fyt {

template <int N_X, int N_Z, int P_NUM>
// 粒子滤波器，目前实现的版本效率不高，可以考虑使用OpenMP等并行计算库进行优化
class ParticleFilter {
  using Particles = Eigen::Matrix<double, N_X, P_NUM>;
  using Weights = Eigen::Matrix<double, P_NUM, 1>;
  using VectorX = Eigen::Matrix<double, N_X, 1>;
  using VectorZ = Eigen::Matrix<double, N_Z, 1>;
  using MatrixXX = Eigen::Matrix<double, N_X, N_X>;
  using MatrixZZ = Eigen::Matrix<double, N_Z, N_Z>;

  using PredictFunc = std::function<VectorX(const VectorX &)>;
  using MeasureFunc = std::function<VectorZ(const VectorX &)>;
  using UpdateQFunc = std::function<MatrixXX()>;
  using UpdateRFunc = std::function<MatrixZZ(const VectorZ &z)>;

public:
  explicit ParticleFilter(const PredictFunc &f,
                          const MeasureFunc &h,
                          const UpdateQFunc &u_q,
                          const UpdateRFunc &u_r) noexcept
  : f(f), h(h), update_q(u_q), update_r(u_r) {
    particles_ = Particles::Zero();
    weights_ = Weights::Zero();
  }

  void initState(const VectorX &x0) noexcept {
    auto noise = generateRandomNoise(update_q());

    particles_ = x0.replicate(1, P_NUM) + noise;
    weights_ = Weights::Ones() / P_NUM;
  }

  void setDim(size_t dim, double val) {
    std::random_device rd;
    auto gen = std::default_random_engine(rd());
    auto process_cov = update_q();
    auto normal_distribution = std::normal_distribution<double>(val, process_cov(dim, dim));

    for (int i = 0; i < P_NUM; ++i) {
      particles_(dim, i) = normal_distribution(gen);
    }
  }

  VectorX predict() {
    Particles pred_particles = Particles::Zero();
    // 根据状态转移方程进行预测
    for (int i = 0; i < P_NUM; i++) {
      pred_particles.col(i) = f(particles_.col(i));
    }
    particles_ = pred_particles;
    return particles_ * weights_;
  }

  VectorX update(const VectorZ &z) {
    Weights w = Weights::Zero();
    MatrixZZ measurement_cov = update_r(z);

    // 基于高斯分布计算权重
    for (int i = 0; i < P_NUM; i++) {
      VectorZ z_hat = h(particles_.col(i));
      double prob = gaussainLikelyhood(z, z_hat, measurement_cov);

      w(i) = prob * weights_(i);
    }

    weights_ = w / w.sum();
    VectorX pred = particles_ * weights_;

    // n_eff: 有效粒子数
    double n_eff = 1.0 / (weights_.transpose() * weights_).value();
    // 有效粒子数小于一半时重采样
    if (n_eff < P_NUM * 0.5) {
      resample();
    }

    return pred;
  }

private:
  void resample() {
    Particles new_particles = Particles::Zero();
    Weights new_weights = Weights::Zero();
    Particles noise = generateRandomNoise(update_q());

    std::random_device rd;
    auto gen = std::default_random_engine(rd());
    // 根据权重进行重采样，每个粒子出现概率等于其权重
    auto discrete_distribution =
      std::discrete_distribution<int>(weights_.data(), weights_.data() + P_NUM);

    std::set<int> idx_set;
    for (int i = 0; i < P_NUM; i++) {
      int idx = discrete_distribution(gen);
      new_particles.col(i) = particles_.col(idx);

      // 只对已经采样过的粒子添加噪声
      // 保证原来的优秀粒子能存活下来，不被噪声覆盖
      if (idx_set.find(idx) != idx_set.end()) {
        new_particles.col(i) += noise.col(i);
      }

      new_weights(i) = 1.0 / P_NUM;
      idx_set.insert(idx);
    }
    particles_ = new_particles;
    weights_ = new_weights;
  }

  template <int dim>
  double gaussainLikelyhood(const Eigen::Matrix<double, dim, 1> &x,
                            const Eigen::Matrix<double, dim, 1> &mean,
                            const Eigen::Matrix<double, dim, dim> &cov) {
    // 在服从均值为mean，协方差为cov的高斯分布下，x出现的概率
    auto diff = x - mean;
    auto exponent = -0.5 * diff.transpose() * cov.inverse() * diff;
    return std::exp(exponent) / std::sqrt(std::pow(2 * M_PI, dim) * cov.determinant());
  }

  // 生成服从高斯分布的随机噪声，假设各个维度之间相互独立
  Particles generateRandomNoise(const MatrixXX &cov) {
    std::random_device rd;
    auto gen = std::default_random_engine(rd());
    auto distributions = std::vector<std::normal_distribution<double>>(N_X);
    for (int i = 0; i < N_X; ++i) {
      distributions[i] = std::normal_distribution<double>(0, cov(i, i));
    }
    Particles noise = Particles::Zero();
    for (int i = 0; i < N_X; ++i) {
      for (int j = 0; j < P_NUM; ++j) {
        noise(i, j) = distributions[i](gen);
      }
    }
    return noise;
  }

private:
  PredictFunc f;
  MeasureFunc h;
  UpdateQFunc update_q;
  UpdateRFunc update_r;
  Particles particles_;
  Weights weights_;
};
}  // namespace fyt
#endif