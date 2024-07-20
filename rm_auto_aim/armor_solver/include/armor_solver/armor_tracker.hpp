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


#ifndef ARMOR_SOLVER_TRACKER_HPP_
#define ARMOR_SOLVER_TRACKER_HPP_

// std
#include <memory>
#include <string>
// ros2
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/vector3.hpp>
// third party
#include <Eigen/Eigen>
// project
#include "rm_interfaces/msg/armors.hpp"
#include "rm_interfaces/msg/target.hpp"
#include "rm_utils/math/extended_kalman_filter.hpp"
#include "armor_solver/motion_model.hpp"

namespace fyt::auto_aim {

enum class ArmorsNum { NORMAL_4 = 4, BALANCE_2 = 2, OUTPOST_3 = 3 };

class Tracker {
public:
  Tracker(double max_match_distance, double max_match_yaw);

  using Armors = rm_interfaces::msg::Armors;
  using Armor = rm_interfaces::msg::Armor;

  void init(const Armors::SharedPtr &armors_msg) noexcept;

  void update(const Armors::SharedPtr &armors_msg) noexcept;

  enum State {
    LOST,
    DETECTING,
    TRACKING,
    TEMP_LOST,
  } tracker_state;

  std::unique_ptr<RobotStateEKF> ekf;

  int tracking_thres;  // frame
  int lost_thres;      // second

  Armor tracked_armor;
  std::string tracked_id;
  ArmorsNum tracked_armors_num;
  Eigen::VectorXd measurement;
  Eigen::VectorXd target_state;

  // To store another pair of armors message
  double d_za, another_r;

  // To store offset relative to the reference plane
  double d_zc;

private:
  void initEKF(const Armor &a) noexcept;

  void handleArmorJump(const Armor &a) noexcept;

  double orientationToYaw(const geometry_msgs::msg::Quaternion &q) noexcept;

  static Eigen::Vector3d getArmorPositionFromState(const Eigen::VectorXd &x) noexcept;

  double max_match_distance_;
  double max_match_yaw_diff_;

  int detect_count_;
  int lost_count_;

  double last_yaw_;
};

}  // namespace fyt::auto_aim

#endif  // ARMOR_SOLVER_ARMOR_TRACKER_HPP_
