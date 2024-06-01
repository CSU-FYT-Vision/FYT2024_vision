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

#include "rm_utils/logger/logger_pool.hpp"

#include "rm_utils/logger/exception.hpp"

namespace fyt::logger {
internal::Logger &LoggerPool::getLogger(const std::string &name) {
  if (auto iter = loggers_.find(name); iter != loggers_.end()) {
    return *iter->second;
  } else {
    throw LoggerNotFoundError(name);
  }
}

void LoggerPool::registerLogger(const std::string &name,
                                const std::string &path,
                                LogLevel level,
                                LogOptions ops) {
  std::lock_guard<std::mutex> lock(l_mutex_);
  if (auto iter = loggers_.find(name); iter == loggers_.end()) {
    loggers_[name] = std::make_shared<internal::Logger>(name, path, level, ops);
  }
}

std::mutex LoggerPool::l_mutex_;
std::unordered_map<std::string, std::shared_ptr<internal::Logger>> LoggerPool::loggers_;
}  // namespace fyt::logger
