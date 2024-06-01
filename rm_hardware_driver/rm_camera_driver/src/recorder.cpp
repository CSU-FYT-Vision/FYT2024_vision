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

#include "rm_camera_driver/recorder.hpp"
// std
#include <filesystem>
// OpenCV
#include <opencv2/opencv.hpp>
// project
#include "rm_utils/logger/log.hpp"

namespace fyt::camera_driver {
Recorder::Recorder(const std::filesystem::path &file, int fps, cv::Size size)
: path(file), size_(size), fps_(fps) {}

void Recorder::start() {
  if (!std::filesystem::exists(path)) {
    std::filesystem::create_directories(path.parent_path());
  }

  writer_.open(path.string(), cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), fps_, size_, true);
  recoring_ = true;
  recorder_thread_ = std::thread(&Recorder::recorderThread, this);
}

Recorder::~Recorder() { stop(); }

void Recorder::addFrame(const Frame &frame) {
  mutex_.lock();
  // Keep the queue size
  if (frame_queue_.size() < 5) {
    frame_queue_.push_back(frame);
  }
  mutex_.unlock();
  cv_.notify_one();
}

void Recorder::stop() {
  recoring_ = false;
  cv_.notify_all();
  recorder_thread_.join();
}

void Recorder::recorderThread() {
  while (recoring_) {
    std::unique_lock<std::mutex> lock(mutex_);
    // !recoring_ is used to break the loop when stop() is called
    cv_.wait(lock, [this] { return !frame_queue_.empty() || !recoring_; });
    if (!recoring_) {
      break;
    }
    auto buffer = std::move(frame_queue_.front());
    frame_queue_.pop_front();
    lock.unlock();
    if (!buffer.empty()) {
      cv::Mat frame(size_, CV_8UC3, buffer.data());
      cv::cvtColor(frame, frame, cv::COLOR_RGB2BGR);
      writer_.write(frame);
    }
  }
}

}  // namespace fyt::camera_driver
