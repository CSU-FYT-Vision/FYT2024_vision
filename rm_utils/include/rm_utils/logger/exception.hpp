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

#ifndef RM_UTILS_LOGGER_EXCEPTION_HPP_
#define RM_UTILS_LOGGER_EXCEPTION_HPP_

#include <fmt/format.h>

namespace fyt::logger {
class LoggerNotFoundError : public std::exception {
public:
  explicit LoggerNotFoundError(std::string_view name) {
    msg = fmt::format("Logger {} Not Found", name);
  }
  const char *what() const noexcept override { return msg.data(); }

private:
  std::string_view msg;
};

class WriteError : public std::exception {
public:
  explicit WriteError(std::string_view name) { msg = fmt::format("Write to {} Error", name); }
  const char *what() const noexcept override { return msg.data(); }

private:
  std::string_view msg;
};

}  // namespace fyt::logger
#endif // RM_UTILS_LOGGER_EXCEPTION_HPP_
