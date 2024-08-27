// Created by Labor 2023.8.25
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

#include "armor_detector/ba_solver.hpp"
// std
#include <memory>
// g2o
#include <g2o/core/robust_kernel.h>
#include <g2o/core/robust_kernel_factory.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/types/slam3d/types_slam3d.h>
// 3rd party
#include <Eigen/Core>
#include <opencv2/core/eigen.hpp>
#include <sophus/se3.hpp>
#include <sophus/so3.hpp>
// project
#include "armor_detector/graph_optimizer.hpp"
#include "armor_detector/types.hpp"
#include "rm_utils/logger/log.hpp"
#include "rm_utils/math/utils.hpp"

namespace fyt::auto_aim {
G2O_USE_OPTIMIZATION_LIBRARY(dense)

BaSolver::BaSolver(std::array<double, 9> &camera_matrix,
                   std::vector<double> &dist_coeffs) {
  K_ = Eigen::Matrix3d::Identity();
  K_(0, 0) = camera_matrix[0];
  K_(1, 1) = camera_matrix[4];
  K_(0, 2) = camera_matrix[2];
  K_(1, 2) = camera_matrix[5];

  // Optimization information
  optimizer_.setVerbose(false);
  // Optimization method
  optimizer_.setAlgorithm(
      g2o::OptimizationAlgorithmFactory::instance()->construct(
          "lm_dense", solver_property_));
  // Initial step size
  lm_algorithm_ = dynamic_cast<g2o::OptimizationAlgorithmLevenberg *>(
      const_cast<g2o::OptimizationAlgorithm *>(optimizer_.algorithm()));
  lm_algorithm_->setUserLambdaInit(0.1);
}

Eigen::Matrix3d
BaSolver::solveBa(const Armor &armor, const Eigen::Vector3d &t_camera_armor,
                  const Eigen::Matrix3d &R_camera_armor,
                  const Eigen::Matrix3d &R_imu_camera) noexcept {
  // Reset optimizer
  optimizer_.clear();

  // Essential coordinate system transformation
  Eigen::Matrix3d R_imu_armor = R_imu_camera * R_camera_armor;
  Sophus::SO3d R_camera_imu = Sophus::SO3d(R_imu_camera.transpose());

  // Compute the initial yaw from rotation matrix
  double initial_armor_yaw;
  auto theta_by_sin = std::asin(-R_imu_armor(0, 1));
  auto theta_by_cos = std::acos(R_imu_armor(1, 1));
  if (std::abs(theta_by_sin) > 1e-5) {
    initial_armor_yaw = theta_by_sin > 0 ? theta_by_cos : -theta_by_cos;
  } else {
    initial_armor_yaw = R_imu_armor(1, 1) > 0 ? 0 : CV_PI;
  }

  // Get the pitch angle of the armor
  double armor_pitch =
      armor.number == "outpost" ? -FIFTTEN_DEGREE_RAD : FIFTTEN_DEGREE_RAD;
  Sophus::SO3d R_pitch = Sophus::SO3d::exp(Eigen::Vector3d(0, armor_pitch, 0));

  // Get the 3D points of the armor
  const auto armor_size =
      armor.type == ArmorType::SMALL
          ? Eigen::Vector2d(SMALL_ARMOR_WIDTH, SMALL_ARMOR_HEIGHT)
          : Eigen::Vector2d(LARGE_ARMOR_WIDTH, LARGE_ARMOR_HEIGHT);
  const auto object_points =
      Armor::buildObjectPoints<Eigen::Vector3d>(armor_size(0), armor_size(1));

  // Fill the optimizer
  size_t id_counter = 0;

  VertexYaw *v_yaw = new VertexYaw();
  v_yaw->setId(id_counter++);
  v_yaw->setEstimate(initial_armor_yaw);
  optimizer_.addVertex(v_yaw);

  const auto &landmarks = armor.landmarks();
  for (size_t i = 0; i < Armor::N_LANDMARKS; i++) {
    g2o::VertexPointXYZ *v_point = new g2o::VertexPointXYZ();
    v_point->setId(id_counter++);
    v_point->setEstimate(Eigen::Vector3d(
        object_points[i].x(), object_points[i].y(), object_points[i].z()));
    v_point->setFixed(true);
    optimizer_.addVertex(v_point);

    EdgeProjection *edge =
        new EdgeProjection(R_camera_imu, R_pitch, t_camera_armor, K_);
    edge->setId(id_counter++);
    edge->setVertex(0, v_yaw);
    edge->setVertex(1, v_point);
    edge->setMeasurement(Eigen::Vector2d(landmarks[i].x, landmarks[i].y));
    edge->setInformation(EdgeProjection::InfoMatrixType::Identity());
    edge->setRobustKernel(new g2o::RobustKernelHuber);
    optimizer_.addEdge(edge);
  }

  // Start optimizing
  optimizer_.initializeOptimization();
  optimizer_.optimize(20);

  // Get yaw angle after optimization
  double yaw_optimized = v_yaw->estimate();

  if (std::isnan(yaw_optimized)) {
    FYT_ERROR("armor_detector", "Yaw angle is nan after optimization");
    return R_camera_armor;
  }

  Sophus::SO3d R_yaw = Sophus::SO3d::exp(Eigen::Vector3d(0, 0, yaw_optimized));
  return (R_camera_imu * R_yaw * R_pitch).matrix();
}

} // namespace fyt::auto_aim