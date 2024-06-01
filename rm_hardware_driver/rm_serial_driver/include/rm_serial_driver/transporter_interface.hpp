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
// Additional modifications and features by Chengfu Zou, 2023.
//
// Copyright (C) FYT Vision Group. All rights reserved.

#ifndef SERIAL_DRIVER_TRANSPORTER_INTERFACE_HPP_
#define SERIAL_DRIVER_TRANSPORTER_INTERFACE_HPP_

// std
#include <memory>
#include <string>

namespace fyt::serial_driver {

// Transporter device interface to transport data between embedded systems
// (stm32,c51) and PC
class TransporterInterface {
public:
  using SharedPtr = std::shared_ptr<TransporterInterface>;
  virtual ~TransporterInterface() = default;
  virtual bool open() = 0;
  virtual void close() = 0;
  virtual bool isOpen() = 0;
  // return recv len>0, return <0 if error
  virtual int read(void *buffer, size_t len) = 0;
  // return send len>0, return <0 if error
  virtual int write(const void *buffer, size_t len) = 0;
  // get error message when open() return false.
  virtual std::string errorMessage() = 0;
};

}  // namespace fyt::serial_driver

#endif  // SERIAL_DRIVER_TRANSPORTER_INTERFACE_HPP_
