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

#include "armor_detector/armor_detector.hpp"
// std
#include <algorithm>
#include <cmath>
#include <execution>
#include <vector>
// OpenCV
#include <opencv2/core.hpp>
#include <opencv2/core/base.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
// project
#include "armor_detector/types.hpp"
#include "rm_utils/common.hpp"

namespace fyt::auto_aim {
Detector::Detector(const int &bin_thres,
                   const EnemyColor &color,
                   const LightParams &l,
                   const ArmorParams &a)
: binary_thres(bin_thres), detect_color(color), light_params(l), armor_params(a) {}

std::vector<Armor> Detector::detect(const cv::Mat &input) noexcept {
  // 1. Preprocess the image
  binary_img = preprocessImage(input);
  // 2. Find lights
  lights_ = findLights(input, binary_img);
  // 3. Match lights to armors
  armors_ = matchLights(lights_);

  if (!armors_.empty() && classifier != nullptr) {
    // Parallel processing
    std::for_each(
      std::execution::par, armors_.begin(), armors_.end(), [this, &input](Armor &armor) {
        // 4. Extract the number image
        armor.number_img = classifier->extractNumber(input, armor);
        // 5. Do classification
        classifier->classify(input, armor);
        // 6. Correct the corners of the armor
        if (corner_corrector != nullptr) {
          corner_corrector->correctCorners(armor, gray_img_);
        }
      });

    // 7. Erase the armors with ignore classes
    classifier->eraseIgnoreClasses(armors_);
  }

  return armors_;
}

cv::Mat Detector::preprocessImage(const cv::Mat &rgb_img) noexcept {
  cv::cvtColor(rgb_img, gray_img_, cv::COLOR_RGB2GRAY);

  cv::Mat binary_img;
  cv::threshold(gray_img_, binary_img, binary_thres, 255, cv::THRESH_BINARY);

  return binary_img;
}

std::vector<Light> Detector::findLights(const cv::Mat &rgb_img,
                                        const cv::Mat &binary_img) noexcept {
  using std::vector;
  vector<vector<cv::Point>> contours;
  vector<cv::Vec4i> hierarchy;
  cv::findContours(binary_img, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);

  vector<Light> lights;
  debug_lights.data.clear();

  for (const auto &contour : contours) {
    if (contour.size() < 6) continue;

    auto light = Light(contour);

    if (isLight(light)) {
      int sum_r = 0, sum_b = 0;
      for (const auto &point : contour) {
        sum_r += rgb_img.at<cv::Vec3b>(point.y, point.x)[0];
        sum_b += rgb_img.at<cv::Vec3b>(point.y, point.x)[2];
      }
      if (std::abs(sum_r - sum_b) / static_cast<int>(contour.size()) >
          light_params.color_diff_thresh) {
        light.color = sum_r > sum_b ? EnemyColor::RED : EnemyColor::BLUE;
      }
      lights.emplace_back(light);
    }
  }
  std::sort(lights.begin(), lights.end(), [](const Light &l1, const Light &l2) {
    return l1.center.x < l2.center.x;
  });
  return lights;
}

bool Detector::isLight(const Light &light) noexcept {
  // The ratio of light (short side / long side)
  float ratio = light.width / light.length;
  bool ratio_ok = light_params.min_ratio < ratio && ratio < light_params.max_ratio;

  bool angle_ok = light.tilt_angle < light_params.max_angle;

  bool is_light = ratio_ok && angle_ok;

  // Fill in debug information
  rm_interfaces::msg::DebugLight light_data;
  light_data.center_x = light.center.x;
  light_data.ratio = ratio;
  light_data.angle = light.tilt_angle;
  light_data.is_light = is_light;
  this->debug_lights.data.emplace_back(light_data);

  return is_light;
}

std::vector<Armor> Detector::matchLights(const std::vector<Light> &lights) noexcept {
  std::vector<Armor> armors;
  this->debug_armors.data.clear();
  // Loop all the pairing of lights
  for (auto light_1 = lights.begin(); light_1 != lights.end(); light_1++) {
    if (light_1->color != detect_color) continue;
    double max_iter_width = light_1->length * armor_params.max_large_center_distance;

    for (auto light_2 = light_1 + 1; light_2 != lights.end(); light_2++) {
      if (light_2->color != detect_color) continue;
      if (containLight(light_1 - lights.begin(), light_2 - lights.begin(), lights)) {
        continue;
      }
      if (light_2->center.x - light_1->center.x > max_iter_width) break;

      auto type = isArmor(*light_1, *light_2);
      if (type != ArmorType::INVALID) {
        auto armor = Armor(*light_1, *light_2);
        armor.type = type;
        armors.emplace_back(armor);
      }
    }
  }

  return armors;
}

// Check if there is another light in the boundingRect formed by the 2 lights
bool Detector::containLight(const int i, const int j, const std::vector<Light> &lights) noexcept {
  const Light &light_1 = lights.at(i), light_2 = lights.at(j);
  auto points = std::vector<cv::Point2f>{light_1.top, light_1.bottom, light_2.top, light_2.bottom};
  auto bounding_rect = cv::boundingRect(points);
  double avg_length = (light_1.length + light_2.length) / 2.0;
  double avg_width = (light_1.width + light_2.width) / 2.0;
  // Only check lights in between
  for (int k = i + 1; k < j; k++) {
    const Light &test_light = lights.at(k);

    // 防止数字干扰
    if (test_light.width > 2 * avg_width) {
      continue;
    }
    // 防止红点准星或弹丸干扰
    if (test_light.length < 0.5 * avg_length) {
      continue;
    }

    if (bounding_rect.contains(test_light.top) || bounding_rect.contains(test_light.bottom) ||
        bounding_rect.contains(test_light.center)) {
      return true;
    }
  }
  return false;
}

ArmorType Detector::isArmor(const Light &light_1, const Light &light_2) noexcept {
  // Ratio of the length of 2 lights (short side / long side)
  float light_length_ratio = light_1.length < light_2.length ? light_1.length / light_2.length
                                                             : light_2.length / light_1.length;
  bool light_ratio_ok = light_length_ratio > armor_params.min_light_ratio;

  // Distance between the center of 2 lights (unit : light length)
  float avg_light_length = (light_1.length + light_2.length) / 2;
  float center_distance = cv::norm(light_1.center - light_2.center) / avg_light_length;
  bool center_distance_ok = (armor_params.min_small_center_distance <= center_distance &&
                             center_distance < armor_params.max_small_center_distance) ||
                            (armor_params.min_large_center_distance <= center_distance &&
                             center_distance < armor_params.max_large_center_distance);

  // Angle of light center connection
  cv::Point2f diff = light_1.center - light_2.center;
  float angle = std::abs(std::atan(diff.y / diff.x)) / CV_PI * 180;
  bool angle_ok = angle < armor_params.max_angle;

  bool is_armor = light_ratio_ok && center_distance_ok && angle_ok;

  // Judge armor type
  ArmorType type;
  if (is_armor) {
    type = center_distance > armor_params.min_large_center_distance ? ArmorType::LARGE
                                                                    : ArmorType::SMALL;
  } else {
    type = ArmorType::INVALID;
  }

  // Fill in debug information
  rm_interfaces::msg::DebugArmor armor_data;
  armor_data.type = armorTypeToString(type);
  armor_data.center_x = (light_1.center.x + light_2.center.x) / 2;
  armor_data.light_ratio = light_length_ratio;
  armor_data.center_distance = center_distance;
  armor_data.angle = angle;
  this->debug_armors.data.emplace_back(armor_data);

  return type;
}

cv::Mat Detector::getAllNumbersImage() const noexcept {
  if (armors_.empty()) {
    return cv::Mat(cv::Size(20, 28), CV_8UC1);
  } else {
    std::vector<cv::Mat> number_imgs;
    number_imgs.reserve(armors_.size());
    for (auto &armor : armors_) {
      number_imgs.emplace_back(armor.number_img);
    }
    cv::Mat all_num_img;
    cv::vconcat(number_imgs, all_num_img);
    return all_num_img;
  }
}

void Detector::drawResults(cv::Mat &img) const noexcept {
  // Draw Lights

  // for (const auto &light : lights_) {
  //   auto line_color =
  //     light.color == EnemyColor::RED ? cv::Scalar(0, 255, 255) : cv::Scalar(255, 255, 0);
  //   // cv::ellipse(img, light, line_color, 2);
  //   cv::line(img, light.top, light.bottom, line_color, 1);
  // }

  // Draw armors
  for (const auto &armor : armors_) {
    // cv::line(img, armor.left_light.top, armor.right_light.bottom, cv::Scalar(0, 255, 0), 1);
    // cv::line(img, armor.left_light.bottom, armor.right_light.top, cv::Scalar(0, 255, 0), 1);

    cv::line(
      img, armor.left_light.top, armor.left_light.bottom, cv::Scalar(0, 255, 0), 1, cv::LINE_AA);
    cv::line(
      img, armor.right_light.bottom, armor.right_light.top, cv::Scalar(0, 255, 0), 1, cv::LINE_AA);
    cv::line(
      img, armor.left_light.top, armor.right_light.top, cv::Scalar(0, 255, 0), 1, cv::LINE_AA);
    cv::line(img,
             armor.right_light.bottom,
             armor.left_light.bottom,
             cv::Scalar(0, 255, 0),
             1,
             cv::LINE_AA);
  }
  // Show numbers and confidence
  for (const auto &armor : armors_) {
    std::string text =
      fmt::format("{} {}", armorTypeToString(armor.type), armor.classfication_result);
    cv::putText(
      img, text, armor.left_light.top, cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 255), 2);
  }
}

}  // namespace fyt::auto_aim
