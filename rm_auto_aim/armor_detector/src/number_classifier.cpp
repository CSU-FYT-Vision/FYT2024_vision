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

// OpenCV
#include <opencv2/core.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/dnn.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
// std
#include <algorithm>
#include <cstddef>
#include <execution>
#include <fstream>
#include <future>
#include <map>
#include <string>
#include <vector>
// 3rd party
#include <fmt/format.h>
// project
#include "armor_detector/number_classifier.hpp"
#include "armor_detector/types.hpp"

namespace fyt::auto_aim {
NumberClassifier::NumberClassifier(const std::string &model_path,
                                   const std::string &label_path,
                                   const double thre,
                                   const std::vector<std::string> &ignore_classes)
: threshold(thre), ignore_classes_(ignore_classes) {
  net_ = cv::dnn::readNetFromONNX(model_path);
  std::ifstream label_file(label_path);
  std::string line;
  while (std::getline(label_file, line)) {
    class_names_.push_back(line);
  }
}

cv::Mat NumberClassifier::extractNumber(const cv::Mat &src, const Armor &armor) const noexcept {
  // Light length in image
  static const int light_length = 12;
  // Image size after warp
  static const int warp_height = 28;
  static const int small_armor_width = 32;
  static const int large_armor_width = 54;
  // Number ROI size
  static const cv::Size roi_size(20, 28);
  static const cv::Size input_size(28, 28);

  // Warp perspective transform
  cv::Point2f lights_vertices[4] = {
    armor.left_light.bottom, armor.left_light.top, armor.right_light.top, armor.right_light.bottom};

  const int top_light_y = (warp_height - light_length) / 2 - 1;
  const int bottom_light_y = top_light_y + light_length;
  const int warp_width = armor.type == ArmorType::SMALL ? small_armor_width : large_armor_width;
  cv::Point2f target_vertices[4] = {
    cv::Point(0, bottom_light_y),
    cv::Point(0, top_light_y),
    cv::Point(warp_width - 1, top_light_y),
    cv::Point(warp_width - 1, bottom_light_y),
  };
  cv::Mat number_image;
  auto rotation_matrix = cv::getPerspectiveTransform(lights_vertices, target_vertices);
  cv::warpPerspective(src, number_image, rotation_matrix, cv::Size(warp_width, warp_height));

  // Get ROI
  number_image = number_image(cv::Rect(cv::Point((warp_width - roi_size.width) / 2, 0), roi_size));

  // Binarize
  cv::cvtColor(number_image, number_image, cv::COLOR_RGB2GRAY);
  cv::threshold(number_image, number_image, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
  cv::resize(number_image, number_image, input_size);
  return number_image;
}

void NumberClassifier::classify(const cv::Mat &src, Armor &armor) noexcept {
  // Normalize
  cv::Mat input = armor.number_img / 255.0;

  // Create blob from image
  cv::Mat blob;
  cv::dnn::blobFromImage(input, blob);

  // Set the input blob for the neural network
  mutex_.lock();
  net_.setInput(blob);

  // Forward pass the image blob through the model
  cv::Mat outputs = net_.forward().clone();
  mutex_.unlock();

  // Decode the output
  double confidence;
  cv::Point class_id_point;
  minMaxLoc(outputs.reshape(1, 1), nullptr, &confidence, nullptr, &class_id_point);
  int label_id = class_id_point.x;

  armor.confidence = confidence;
  armor.number = class_names_[label_id];

  armor.classfication_result = fmt::format("{}:{:.1f}%", armor.number, armor.confidence * 100.0);
}

void NumberClassifier::eraseIgnoreClasses(std::vector<Armor> &armors) noexcept {
  armors.erase(
    std::remove_if(armors.begin(),
                   armors.end(),
                   [this](const Armor &armor) {
                     if (armor.confidence < threshold) {
                       return true;
                     }

                     for (const auto &ignore_class : ignore_classes_) {
                       if (armor.number == ignore_class) {
                         return true;
                       }
                     }

                     bool mismatch_armor_type = false;
                     if (armor.type == ArmorType::LARGE) {
                       mismatch_armor_type = armor.number == "outpost" || armor.number == "2" ||
                                             armor.number == "sentry";
                     } else if (armor.type == ArmorType::SMALL) {
                       mismatch_armor_type = armor.number == "1" || armor.number == "base";
                     }
                     return mismatch_armor_type;
                   }),
    armors.end());
}

}  // namespace fyt::auto_aim
