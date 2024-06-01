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

#ifndef ARMOR_DETECTOR_DETECTOR_HPP_
#define ARMOR_DETECTOR_DETECTOR_HPP_

// std
#include <cmath>
#include <rm_utils/common.hpp>
#include <string>
#include <vector>
// third party 
#include <opencv2/core.hpp>
#include <opencv2/core/types.hpp>
// project
#include "armor_detector/light_corner_corrector.hpp"
#include "armor_detector/types.hpp"
#include "armor_detector/number_classifier.hpp"
#include "rm_interfaces/msg/debug_armors.hpp"
#include "rm_interfaces/msg/debug_lights.hpp"

namespace fyt::auto_aim {
class Detector {
public:
  struct LightParams {
    // width / height
    double min_ratio;
    double max_ratio;
    // vertical angle
    double max_angle;
    // judge color
    int color_diff_thresh;
  };

  struct ArmorParams {
    double min_light_ratio;
    // light pairs distance
    double min_small_center_distance;
    double max_small_center_distance;
    double min_large_center_distance;
    double max_large_center_distance;
    // horizontal angle
    double max_angle;
  };

  Detector(const int &bin_thres, const EnemyColor &color, const LightParams &l,
           const ArmorParams &a);

  std::vector<Armor> detect(const cv::Mat &input) noexcept;

  cv::Mat preprocessImage(const cv::Mat &input) noexcept;
  std::vector<Light> findLights(const cv::Mat &rbg_img,
                                const cv::Mat &binary_img) noexcept;
  std::vector<Armor> matchLights(const std::vector<Light> &lights) noexcept;

  // For debug usage
  cv::Mat getAllNumbersImage() const noexcept;
  void drawResults(cv::Mat &img) const noexcept;

  // Parameters
  int binary_thres;
  EnemyColor detect_color;
  LightParams light_params;
  ArmorParams armor_params;

  std::unique_ptr<NumberClassifier> classifier;
  std::unique_ptr<LightCornerCorrector> corner_corrector;

  // Debug msgs
  cv::Mat binary_img;
  rm_interfaces::msg::DebugLights debug_lights;
  rm_interfaces::msg::DebugArmors debug_armors;

private:
  bool isLight(const Light &possible_light) noexcept;
  bool containLight(const int i,const int j,const std::vector<Light> &lights) noexcept;
  ArmorType isArmor(const Light &light_1, const Light &light_2) noexcept;

  cv::Mat gray_img_;

  std::vector<Light> lights_;
  std::vector<Armor> armors_;
};

} // namespace fyt::auto_aim

#endif // ARMOR_DETECTOR_DETECTOR_HPP_
