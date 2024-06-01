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

#include "armor_detector/graph_optimizer.hpp"
// std
#include <algorithm>
// third party
#include <Eigen/Core>
// project
#include "armor_detector/types.hpp"
#include "rm_utils/math/utils.hpp"

namespace fyt::auto_aim {

void VertexYaw::oplusImpl(const double *update) {
  _estimate += Eigen::Vector<double, 1>(update[0]);
}

EdgeProjection::EdgeProjection(const Sophus::SO3d &r,
                               const Eigen::Vector3d &t,
                               const CameraInternalK &k,
                               const Eigen::Vector2d &size,
                               const double pitch)
: camera2imu_(r), t_(t), K_(k), pitch_(pitch) {
  object_points_ = Armor::buildObjectPoints<Eigen::Vector3d>(size(0), size(1));
}

void EdgeProjection::computeError() {
  const VertexYaw *v_yaw = static_cast<VertexYaw *>(_vertices[0]);

  std::array<double, 3> euler = {0, pitch_, v_yaw->estimate()(0)};
  Eigen::Matrix3d imu2armor = utils::eulerToMatrix(euler, utils::EulerOrder::XYZ);

  Eigen::Matrix3d R = camera2imu_.matrix() * imu2armor;
  Eigen::Matrix3d K = K_.toMatrix();

  Eigen::Vector<double, Armor::N_LANDMARKS_2> landmarks_predicted;
  // Calculate the error of every point
  for (int i = 0; i < Armor::N_LANDMARKS; i++) {
    // Project the point to the image plane
    auto p = object_points_[i];
    p = R * p + t_;
    p = K * (p / p.z());

    landmarks_predicted(i * 2) = p.x();
    landmarks_predicted(i * 2 + 1) = p.y();
  }
  _error = landmarks_predicted - _measurement;
}

}  // namespace fyt::auto_aim
