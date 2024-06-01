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

#ifndef RM_UTILS_LOGGER_POOL_HPP_
#define RM_UTILS_LOGGER_POOL_HPP_

#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>

#include "rm_utils/logger/impl/logger_impl.hpp"

namespace fyt::logger {
class LoggerPool {
public:
  static internal::Logger &getLogger(const std::string &name);

  static void registerLogger(const std::string &name,
                             const std::string &path,
                             LogLevel level,
                             LogOptions = DEFAULT_OPTIONS);

private:
  LoggerPool() = default;
  ~LoggerPool() = default;
  LoggerPool(const LoggerPool &) = delete;
  LoggerPool &operator=(const LoggerPool &) = delete;
  LoggerPool(LoggerPool &&) = delete;

  static std::mutex l_mutex_;
  static std::unordered_map<std::string, std::shared_ptr<internal::Logger>> loggers_;
};
}  // namespace fyt::logger
#endif // RM_UTILS?LOGGER_POOL_HPP_
