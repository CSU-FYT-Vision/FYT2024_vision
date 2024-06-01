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

#ifndef ARMOR_DETECTOR_NUMBER_CLASSIFIER_HPP_
#define ARMOR_DETECTOR_NUMBER_CLASSIFIER_HPP_

// std
#include <cstddef>
#include <iostream>
#include <map>
#include <string>
#include <vector>
// third party
#include <opencv2/opencv.hpp>
// project
#include "armor_detector/types.hpp"

namespace fyt::auto_aim {
// Class used to classify the number of the armor, based on the MLP model
class NumberClassifier {
public:
  NumberClassifier(const std::string &model_path,
                   const std::string &label_path,
                   const double threshold,
                   const std::vector<std::string> &ignore_classes = {});

  // Extract the roi image of number from the src
  cv::Mat extractNumber(const cv::Mat &src, const Armor &armor) const noexcept;

  // Classify the number of the armor
  void classify(const cv::Mat &src, Armor &armor) noexcept;

  // Erase the ignore classes
  void eraseIgnoreClasses(std::vector<Armor> &armors) noexcept;

  double threshold;

private:
  std::mutex mutex_;
  cv::dnn::Net net_;
  std::vector<std::string> class_names_;
  std::vector<std::string> ignore_classes_;
};
}  // namespace fyt::auto_aim
#endif  // ARMOR_DETECTOR_NUMBER_CLASSIFIER_HPP_
