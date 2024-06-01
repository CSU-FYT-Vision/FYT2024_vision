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

#ifndef RM_UTILS_LOGGER_TYPES_HPP_
#define RM_UTILS_LOGGER_TYPES_HPP_

// std
#include <algorithm>
#include <cstdint>
#include <exception>
#include <mutex>
#include <unordered_map>

// fmt
#include <fmt/color.h>
#include <fmt/core.h>

namespace fyt::logger {

enum class LogLevel : std::uint8_t { DEBUG, INFO, WARN, ERROR, FATAL };

constexpr const char *LogNameTable[5] = {"DEBUG", "INFO", "WARN", "ERROR", "FATAL"};

// DEBUG = gray, INFO = white, WARN = yellow, ERROR = red, FATAL = blue
constexpr const char *LogColorTable[5] = {"<font color=\"#9B9B9B\">{}</font>",
                                          "<font color=\"#FFFFFF\">{}</font>",
                                          "<font color=\"#FFFF00\">{}</font>",
                                          "<font color=\"#FF0000\">{}</font>",
                                          "<font color=\"#0000FF\">{}</font>"};

constexpr fmt::color LogFmtColorTable[5] = {
  fmt::color::gray, fmt::color::white, fmt::color::yellow, fmt::color::red, fmt::color::blue};

using LogOptions = unsigned char;

// default options: without date named directory, without date named suffix, append to file
#define DEFAULT_OPTIONS 0b00000000
#define DATE_DIR 0b00000001
#define DATE_SUFFIX 0b00000010
#define OVER_WRITE 0b00000100

}  // namespace fyt::logger
#endif  // RM_UTILS_LOGGER_TYPES_HPP_
 