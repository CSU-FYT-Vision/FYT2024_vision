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

#ifndef RM_UTILS_ASSERT_HPP_
#define RM_UTILS_ASSERT_HPP_

#include <iostream>
#include <sstream>

#define FYT_ASSERT(condition)                            \
  do {                                                   \
    if (!(condition)) {                                  \
      std::ostringstream oss;                            \
      oss << "Assertion failed: (" << #condition << ")"; \
      std::cerr << oss.str() << std::endl;               \
      std::abort();                                      \
    }                                                    \
  } while (0)

#define FYT_ASSERT_MSG(condition, msg)                           \
  do {                                                           \
    if (!(condition)) {                                          \
      std::ostringstream oss;                                    \
      oss << "Assertion failed: (" << #condition << ") " << msg; \
      std::cerr << oss.str() << std::endl;                       \
      std::abort();                                              \
    }                                                            \
  } while (0)

#endif