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

#include "rm_utils/math/manual_compensator.hpp"

namespace fyt {
bool ManualCompensator::updateMap(const LineRegion& d_region,
                                  const LineRegion& h_region,
                                  const double pitch_offset,
                                  const double yaw_offset) {
    auto target_dist_node = 
      std::find_if(angle_offset_map_.begin(), 
                   angle_offset_map_.end(),
                   [&](const DistMapNode& dist_node) {
     return dist_node.dist_region.checkIntersection(d_region); 
    });

    if (target_dist_node == angle_offset_map_.end()) {
      HeightMapNode height_node(h_region, pitch_offset, yaw_offset);
      std::vector<HeightMapNode> h_nodes{height_node};
      DistMapNode dist_node(d_region, h_nodes);
      angle_offset_map_.emplace_back(dist_node);
    } else {
      auto target_height_node = 
        std::find_if(target_dist_node->height_map.begin(),
                     target_dist_node->height_map.end(),
                     [&](const HeightMapNode& height_node) {
      return height_node.height_region.checkIntersection(h_region);
      });

      if (target_height_node == target_dist_node->height_map.end()) {
        HeightMapNode height_node(h_region, pitch_offset, yaw_offset);
        target_dist_node->height_map.emplace_back(height_node);
      } else {
        return false;
      }
    }
    return true;
  }

std::vector<double> ManualCompensator::angleHardCorrect(const double dist, 
                                           const double height) {
  auto target_dist_node = 
    std::find_if(angle_offset_map_.begin(), 
                  angle_offset_map_.end(),
                  [&](const DistMapNode& dist_node) {
    return dist_node.dist_region.checkPoint(dist); 
  });

  if (target_dist_node != angle_offset_map_.end()) {
    auto target_height_node = 
      std::find_if(target_dist_node->height_map.begin(),
                    target_dist_node->height_map.end(),
                    [&](const HeightMapNode& height_node) {
      return height_node.height_region.checkPoint(height);
    });

    if (target_height_node != target_dist_node->height_map.end()) {
      return {target_height_node->pitch_offset, target_height_node->yaw_offset};    
    }
  }
  return {0.0, 0.0};
} 
  
bool ManualCompensator::parseStr(const std::string& str, 
                                 std::vector<double>& nums) {
  std::stringstream ss(str);
  double num;
  while (!ss.eof()) {
    ss >> num;
    nums.emplace_back(num);
  }
  
  if (nums.size() != NORMAL_STR_NUM) {
    return false;
  }
  return true;
}

bool ManualCompensator::updateMapByStr(const std::string &str) {
  std::vector<double> nums;

  if (!parseStr(str, nums)) {
    return false;
  }

  LineRegion d_region(nums[0], nums[1]);
  LineRegion h_region(nums[2], nums[3]);
  if (!updateMap(d_region, h_region, nums[4], nums[5])) {
    return false;
  }
  return true;
}
}  // namespace fyt