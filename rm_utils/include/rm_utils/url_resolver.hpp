// Copyright (C) 2021 RoboMaster-OSS
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
//
// Additional modifications and features by Chengfu Zou, 2024.
//
// Copyright (C) FYT Vision Group. All rights reserved.

#ifndef RM_UTILS_URL_RESOLVER_HPP_
#define RM_UTILS_URL_RESOLVER_HPP_

#include <filesystem>
#include <string>

namespace fyt::utils {
class URLResolver {
public:
  static std::filesystem::path getResolvedPath(const std::string &url);

private:
  static std::string resolveUrl(const std::string &url);

  URLResolver() = delete;

  enum class UrlType {
    EMPTY = 0,  // empty string
    FILE,       // file
    PACKAGE,    // package
    INVALID,    // anything >= is invalid
  };
  static UrlType parseUrl(const std::string &url);

  static std::string getPackageFileName(const std::string &url);
};
}  // namespace fyt::utils

#endif // RM_UTILS_URL_RESOLVER_HPP_
