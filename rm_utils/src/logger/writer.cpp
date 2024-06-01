// Created by Chengfu Zou
// Copyright (c) FYT Vision Group. All rights reserved.
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

#include "rm_utils/logger/impl/writer.hpp"

#include <filesystem>
#include <fstream>

#include "rm_utils/logger/impl/global_mutex.hpp"

namespace fyt::logger {

Writer::Writer(const std::string &filename) : r_mutex_(GlobalMutex::getFileMutex(filename)) {
  auto parent_path = std::filesystem::path(filename).parent_path();
  parent_path = parent_path.empty() ? "." : parent_path;
  if (!std::filesystem::exists(parent_path)) {
    std::filesystem::create_directories(parent_path);
  }
  file_.open(filename, std::ios::out | std::ios::app);
}

void Writer::write(const std::string &message) {
  std::lock_guard<std::mutex> lock(r_mutex_);
  file_ << message << "\n\n";
  file_.flush();
}

void Writer::flush() {
  std::lock_guard<std::mutex> lock(r_mutex_);
  file_.flush();
}

Writer::~Writer() {
  std::lock_guard<std::mutex> lock(r_mutex_);
  file_.close();
}

}  // namespace fyt::logger
