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

#ifndef RM_UTILS_LOGGER_WRITER_HPP_
#define RM_UTILS_LOGGER_WRITER_HPP_

// std
#include <fstream>
#include <mutex>

namespace fyt::logger {

class Writer {
public:
  explicit Writer(const std::string &filename);

  ~Writer();

  void write(const std::string &message);

  void flush();

private:
  std::ofstream file_;
  std::mutex &r_mutex_;
};
}  // namespace fyt::logger
#endif  // RM_UTILS_LOGGER_WRITER_HPP_ 
