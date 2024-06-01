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

#ifndef RM_UTILS_LOGGER_GLOBAL_MUTEX_HPP_
#define RM_UTILS_LOGGER_GLOBAL_MUTEX_HPP_

#include <mutex>
#include <unordered_map>

namespace fyt::logger {

static std::mutex g_mutex_;

class GlobalMutex {
public:
  inline static std::mutex &getConsoleMutex() {
    static std::mutex s_mutex;
    return s_mutex;
  }

  inline static std::mutex &getFileMutex(const std::string &filename) {
    static std::unordered_map<std::string, std::mutex> file_mutex_map;
    std::lock_guard<std::mutex> lock(g_mutex_);
    return file_mutex_map[filename];
  }

private:
  GlobalMutex() = default;
  ~GlobalMutex() = default;
};
}  // namespace fyt::logger
#endif
