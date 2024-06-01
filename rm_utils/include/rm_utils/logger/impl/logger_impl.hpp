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

#ifndef RM_UTILS_LOGGER_LOGGER_IMPL_HPP_
#define RM_UTILS_LOGGER_LOGGER_IMPL_HPP_

// std
#include <array>
#include <memory>
#include <stdexcept>
#include <string>
#include <unordered_map>

// fmt
#include <fmt/color.h>
#include <fmt/core.h>

// project
#include "rm_utils/logger/impl/writer.hpp"
#include "rm_utils/logger/types.hpp"

namespace fyt::logger {

namespace internal {
class Logger {
public:
  Logger(std::string name, std::string path, LogLevel level, LogOptions ops);

  template <typename... Args>
  void log(LogLevel level, const std::string &format, Args... args) {
    std::string log_info = fmt::format(format, args...);
    std::string level_prefix = fmt::format("[{}] ", LogNameTable[static_cast<std::uint8_t>(level)]);
    std::string name_prefix = fmt::format("[{}] ", name_);
    std::string log_time = fmt::format("[{}] ", getLocalTime());
    std::string message =
      fmt::format("{} {} {}: {}", level_prefix, name_prefix, log_time, log_info);

    if (level >= this->level_) {
      std::string colored_message =
        fmt::format(LogColorTable[static_cast<std::uint8_t>(level)], message);
      writer_->write(colored_message);
    }
    std::lock_guard<std::mutex> lock(consle_mutex_);
    fmt::print(fg(LogFmtColorTable[static_cast<std::uint8_t>(level)]), "{}\n", message);
  }

  template <typename... Args>
  void debug(const std::string &format, Args... args) {
    log(LogLevel::DEBUG, format, args...);
  }

  template <typename... Args>
  void info(const std::string &format, Args... args) {
    log(LogLevel::INFO, format, args...);
  }

  template <typename... Args>
  void warn(const std::string &format, Args... args) {
    log(LogLevel::WARN, format, args...);
  }

  template <typename... Args>
  void error(const std::string &format, Args... args) {
    log(LogLevel::ERROR, format, args...);
  }

  template <typename... Args>
  void fatal(const std::string &format, Args... args) {
    log(LogLevel::FATAL, format, args...);
  }

  template <typename... Args>
  void print(const std::string &format, Args... args) {
    std::lock_guard<std::mutex> lock(consle_mutex_);
    fmt::print(format, args...);
  }

  void setLevel(LogLevel level);

  void flush();

private:
  std::string getLocalTime();

  std::mutex &consle_mutex_;
  std::string name_;
  LogLevel level_;
  std::unique_ptr<Writer> writer_;
};
}  // namespace internal

}  // namespace fyt::logger
#endif  // RM_UTILS_LOGGER_LOGGER_IMPL_HPP_ 
