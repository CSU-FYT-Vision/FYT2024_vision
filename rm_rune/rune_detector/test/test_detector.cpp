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
#include "rm_utils/common.hpp"
#include "rm_utils/url_resolver.hpp"
#include "rune_detector/rune_detector.hpp"
#include "rune_detector/types.hpp"

using namespace fyt;
using namespace fyt::rune;
TEST(RuneDetectorNodeTest, NodeStartupTest) {
  // Init Params
  std::string model_path = "package://rune_detector/model/yolox_rune_3.6m.onnx";
  std::string device_type = "AUTO";

  float conf_threshold = 0.5;
  int top_k = 128;
  float nms_threshold = 0.3;

  namespace fs = std::filesystem;
  fs::path resolved_path = utils::URLResolver::getResolvedPath(model_path);

  // Create detector
  auto rune_detector = std::make_unique<RuneDetector>(
    resolved_path, device_type, conf_threshold, top_k, nms_threshold);

  // Set detect callback
  std::vector<RuneObject> runes;
  rune_detector->setCallback([&runes](std::vector<RuneObject> &objs,
                                      int64_t timestamp_nanosec,
                                      const cv::Mat &src_img) { runes = objs; });

  // init detector
  rune_detector->init();

  // Load test image
  fs::path test_image_path =
    utils::URLResolver::getResolvedPath("package://rune_detector/docs/test.png");
  cv::Mat test_image = cv::imread(test_image_path.string(), cv::IMREAD_COLOR);
  cv::cvtColor(test_image, test_image, cv::COLOR_BGR2RGB);

  auto future = rune_detector->pushInput(test_image, 0);
  future.get();

  EXPECT_EQ(runes.size(), static_cast<size_t>(3));
  std::sort(runes.begin(), runes.end(), [](const RuneObject &a, const RuneObject &b) {
    return a.type < b.type;
  });
  EXPECT_EQ(runes[0].type, RuneType::INACTIVATED);
  EXPECT_EQ(runes[1].type, RuneType::ACTIVATED);
  EXPECT_EQ(runes[2].type, RuneType::ACTIVATED);
}
