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

#include "rm_utils/logger/impl/logger_impl.hpp"

#include <chrono>
#include <iomanip>
#include <string>
#include <utility>

#include "rm_utils/logger/impl/global_mutex.hpp"
#include "rm_utils/logger/types.hpp"

namespace fyt::logger {

namespace internal {

#define CHECK_OPTION(OPS, OPTION) ((OPS & OPTION) == OPTION)

Logger::Logger(std::string name, std::string filename, LogLevel level, LogOptions ops)
: consle_mutex_(GlobalMutex::getConsoleMutex()), name_(name), level_(level) {
  std::string cur_date = getLocalTime().substr(0, 10);

  if (filename.empty()) {
    filename = "./";
  }
  if (filename.back() != '/') {
    filename.append("/");
  }
  if (filename.front() == '~') {
    std::string home_path = std::getenv("HOME");
    filename.replace(0, 1, home_path);
  }

  if (CHECK_OPTION(ops, DATE_DIR)) {
    filename.append(cur_date + "/");
  }
  if (CHECK_OPTION(ops, DATE_SUFFIX)) {
    filename.append(name + "_" + cur_date + ".log.md");
  } else {
    filename.append(name + ".log.md");
  }

  writer_ = std::make_unique<Writer>(filename);
}

void Logger::setLevel(LogLevel level) { level_ = level; }

void Logger::flush() { writer_->flush(); }

std::string Logger::getLocalTime() {
  std::time_t tt = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
  // std::tm tm = *std::gmtime(&tt);  //GMT (UTC)
  std::tm tm = *std::localtime(&tt);  //Locale time-zone, usually UTC by default.
  std::stringstream ss;
  ss << std::put_time(&tm, "%Y-%m-%d %H:%M:%S");
  return ss.str();
}
}  // namespace internal
}  // namespace fyt::logger
