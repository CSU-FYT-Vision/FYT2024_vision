// Created by Chengfu Zou
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

#ifndef RM_CAMERA_DRIVER_RECORDER_HPP_
#define RM_CAMERA_DRIVER_RECORDER_HPP_

// std
#include <atomic>
#include <condition_variable>
#include <deque>
#include <filesystem>
#include <mutex>
#include <thread>
#include <vector>
// OpenCV
#include <opencv2/videoio.hpp>

namespace fyt::camera_driver {
class Recorder {
public:
  using Frame = std::vector<unsigned char>;
  Recorder(const std::filesystem::path &file, int fps, cv::Size size);
  ~Recorder();

  void addFrame(const Frame &frame);
  void start();
  void stop();

  std::filesystem::path path;

private:
  void recorderThread();

  cv::Size size_;
  int fps_;
  cv::VideoWriter writer_;

  std::deque<Frame> frame_queue_;

  std::mutex mutex_;
  std::atomic<bool> recoring_;
  std::condition_variable cv_;
  std::thread recorder_thread_;
};
}  // namespace fyt::camera_driver
#endif  // RM_CAMERA_DRIVER_RECORDER_HPP_
