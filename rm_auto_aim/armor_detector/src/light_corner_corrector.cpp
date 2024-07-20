// Maintained by Shenglin Qin, Chengfu Zou
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

#include "armor_detector/light_corner_corrector.hpp"

#include <numeric>

namespace fyt::auto_aim {

void LightCornerCorrector::correctCorners(Armor &armor, const cv::Mat &gray_img) {
  // If the width of the light is too small, the correction is not performed
  constexpr int PASS_OPTIMIZE_WIDTH = 3;

  if (armor.left_light.width > PASS_OPTIMIZE_WIDTH) {
    // Find the symmetry axis of the light
    SymmetryAxis left_axis = findSymmetryAxis(gray_img, armor.left_light);
    armor.left_light.center = left_axis.centroid;
    armor.left_light.axis = left_axis.direction;
    // Find the corner of the light
    if (cv::Point2f t = findCorner(gray_img, armor.left_light, left_axis, "top"); t.x > 0) {
      armor.left_light.top = t;
    }
    if (cv::Point2f b = findCorner(gray_img, armor.left_light, left_axis, "bottom"); b.x > 0) {
      armor.left_light.bottom = b;
    }
  }

  if (armor.right_light.width > PASS_OPTIMIZE_WIDTH) {
    // Find the symmetry axis of the light
    SymmetryAxis right_axis = findSymmetryAxis(gray_img, armor.right_light);
    armor.right_light.center = right_axis.centroid;
    armor.right_light.axis = right_axis.direction;
    // Find the corner of the light
    if (cv::Point2f t = findCorner(gray_img, armor.right_light, right_axis, "top"); t.x > 0) {
      armor.right_light.top = t;
    }
    if (cv::Point2f b = findCorner(gray_img, armor.right_light, right_axis, "bottom"); b.x > 0) {
      armor.right_light.bottom = b;
    }
  }
}

SymmetryAxis LightCornerCorrector::findSymmetryAxis(const cv::Mat &gray_img, const Light &light) {
  constexpr float MAX_BRIGHTNESS = 25;
  constexpr float SCALE = 0.07;

  // Scale the bounding box
  cv::Rect light_box = light.boundingRect();
  light_box.x -= light_box.width * SCALE;
  light_box.y -= light_box.height * SCALE;
  light_box.width += light_box.width * SCALE * 2;
  light_box.height += light_box.height * SCALE * 2;

  // Check boundary
  light_box.x = std::max(light_box.x, 0);
  light_box.x = std::min(light_box.x, gray_img.cols - 1);
  light_box.y = std::max(light_box.y, 0);
  light_box.y = std::min(light_box.y, gray_img.rows - 1);
  light_box.width = std::min(light_box.width, gray_img.cols - light_box.x);
  light_box.height = std::min(light_box.height, gray_img.rows - light_box.y);

  // Get normalized light image
  cv::Mat roi = gray_img(light_box);
  float mean_val = cv::mean(roi)[0];
  roi.convertTo(roi, CV_32F);
  cv::normalize(roi, roi, 0, MAX_BRIGHTNESS, cv::NORM_MINMAX);

  // Calculate the centroid
  cv::Moments moments = cv::moments(roi, false);
  cv::Point2f centroid = cv::Point2f(moments.m10 / moments.m00, moments.m01 / moments.m00) +
                         cv::Point2f(light_box.x, light_box.y);

  // Initialize the PointCloud
  std::vector<cv::Point2f> points;
  for (int i = 0; i < roi.rows; i++) {
    for (int j = 0; j < roi.cols; j++) {
      for (int k = 0; k < std::round(roi.at<float>(i, j)); k++) {
        points.emplace_back(cv::Point2f(j, i));
      }
    }
  }
  cv::Mat points_mat = cv::Mat(points).reshape(1);

  // PCA (Principal Component Analysis)
  auto pca = cv::PCA(points_mat, cv::Mat(), cv::PCA::DATA_AS_ROW);

  // Get the symmetry axis
  cv::Point2f axis =
    cv::Point2f(pca.eigenvectors.at<float>(0, 0), pca.eigenvectors.at<float>(0, 1));

  // Normalize the axis
  axis = axis / cv::norm(axis);

  if (axis.y > 0) {
    axis = -axis;
  }

  return SymmetryAxis{.centroid = centroid, .direction = axis, .mean_val = mean_val};
}

cv::Point2f LightCornerCorrector::findCorner(const cv::Mat &gray_img,
                                             const Light &light,
                                             const SymmetryAxis &axis,
                                             std::string order) {
  constexpr float START = 0.8 / 2;
  constexpr float END = 1.2 / 2;

  auto inImage = [&gray_img](const cv::Point &point) -> bool {
    return point.x >= 0 && point.x < gray_img.cols && point.y >= 0 && point.y < gray_img.rows;
  };

  auto distance = [](float x0, float y0, float x1, float y1) -> float {
    return std::sqrt((x0 - x1) * (x0 - x1) + (y0 - y1) * (y0 - y1));
  };

  int oper = order == "top" ? 1 : -1;
  float L = light.length;
  float dx = axis.direction.x * oper;
  float dy = axis.direction.y * oper;

  std::vector<cv::Point2f> candidates;

  // Select multiple corner candidates and take the average as the final corner
  int n = light.width - 2;
  int half_n = std::round(n / 2);
  for (int i = -half_n; i <= half_n; i++) {
    float x0 = axis.centroid.x + L * START * dx + i;
    float y0 = axis.centroid.y + L * START * dy;

    cv::Point2f prev = cv::Point2f(x0, y0);
    cv::Point2f corner = cv::Point2f(x0, y0);
    float max_brightness_diff = 0;
    bool has_corner = false;
    // Search along the symmetry axis to find the corner that has the maximum brightness difference
    for (float x = x0 + dx, y = y0 + dy; distance(x, y, x0, y0) < L * (END - START);
         x += dx, y += dy) {
      cv::Point2f cur = cv::Point2f(x, y);
      if (!inImage(cv::Point(cur))) {
        break;
      }

      float brightness_diff = gray_img.at<uchar>(prev) - gray_img.at<uchar>(cur);
      if (brightness_diff > max_brightness_diff && gray_img.at<uchar>(prev) > axis.mean_val) {
        max_brightness_diff = brightness_diff;
        corner = prev;
        has_corner = true;
      }

      prev = cur;
    }

    if (has_corner) {
      candidates.emplace_back(corner);
    }
  }
  if (!candidates.empty()) {
    cv::Point2f result = std::accumulate(candidates.begin(), candidates.end(), cv::Point2f(0, 0));
    return result / static_cast<float>(candidates.size());
  }

  return cv::Point2f(-1, -1);
}

}  // namespace fyt::auto_aim
