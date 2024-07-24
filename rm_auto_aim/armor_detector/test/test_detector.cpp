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

// gest
#include <gtest/gtest.h>
// ros2
#include <opencv2/imgproc.hpp>
#include <rclcpp/executors.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/utilities.hpp>
// std
#include <memory>
// opencv
#include <opencv2/opencv.hpp>
// project
#include "armor_detector/armor_detector.hpp"
#include "rm_utils/common.hpp"
#include "rm_utils/url_resolver.hpp"

using namespace fyt;
using namespace fyt::auto_aim;
TEST(ArmorDetectorNodeTest, NodeStartupTest) {
  // Init detector
  int binary_thres = 160;
  Detector::LightParams l_params = {
    .min_ratio = 0.08, .max_ratio = 0.4, .max_angle = 40.0, .color_diff_thresh = 25};
  Detector::ArmorParams a_params = {.min_light_ratio = 0.6,
                                    .min_small_center_distance = 0.8,
                                    .max_small_center_distance = 3.2,
                                    .min_large_center_distance = 3.2,
                                    .max_large_center_distance = 5.0,
                                    .max_angle = 35.0};

  auto detector = std::make_unique<Detector>(binary_thres, EnemyColor::RED, l_params, a_params);

  // Init classifier
  namespace fs = std::filesystem;
  fs::path model_path =
    utils::URLResolver::getResolvedPath("package://armor_detector/model/lenet.onnx");
  fs::path label_path =
    utils::URLResolver::getResolvedPath("package://armor_detector/model/label.txt");
  detector->classifier = std::make_unique<NumberClassifier>(
    model_path, label_path, 0.6, std::vector<std::string>{"negative"});

  // Load test image
  fs::path test_image_path =
    utils::URLResolver::getResolvedPath("package://armor_detector/docs/test.png");
  cv::Mat test_image = cv::imread(test_image_path.string(), cv::IMREAD_COLOR);
  cv::cvtColor(test_image, test_image, cv::COLOR_BGR2RGB);

  // Detect
  std::vector<Armor> armors = detector->detect(test_image);

  std::sort(armors.begin(), armors.end(), [](const Armor &a, const Armor &b) {
    return a.number < b.number;
  });

  EXPECT_EQ(armors.size(), static_cast<size_t>(6));
  EXPECT_EQ(armors[0].number, "2");
  EXPECT_EQ(armors[1].number, "3");
  EXPECT_EQ(armors[2].number, "4");
  EXPECT_EQ(armors[3].number, "5");
  EXPECT_EQ(armors[4].number, "outpost");
  EXPECT_EQ(armors[5].number, "sentry");
}
