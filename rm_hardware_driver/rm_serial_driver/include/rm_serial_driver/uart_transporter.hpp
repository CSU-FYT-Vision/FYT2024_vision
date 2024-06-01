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

#ifndef SERIAL_DRIVER_UART_TRANSPORTER_HPP_
#define SERIAL_DRIVER_UART_TRANSPORTER_HPP_

// std
#include <string>
// project
#include "rm_serial_driver/transporter_interface.hpp"

namespace fyt::serial_driver {

// 串口数据传输设备，符合通用传输接口。
class UartTransporter : public TransporterInterface {
public:
  UartTransporter(const std::string &device_path = "/dev/ttyUSB0",
                  int speed = 115200,
                  int flow_ctrl = 0,
                  int databits = 8,
                  int stopbits = 1,
                  int parity = 'N')
  : device_path_(device_path)
  , speed_(speed)
  , flow_ctrl_(flow_ctrl)
  , databits_(databits)
  , stopbits_(stopbits)
  , parity_(parity) {}

  bool open() override;
  void close() override;
  bool isOpen() override;
  int read(void *buffer, size_t len) override;
  int write(const void *buffer, size_t len) override;
  std::string errorMessage() override { return error_message_; }

private:
  bool setParam(
    int speed = 115200, int flow_ctrl = 0, int databits = 0, int stopbits = 1, int parity = 'N');

private:
  // 设备文件描述符
  int fd_{-1};
  // 设备状态
  bool is_open_{false};
  std::string error_message_;
  // 设备参数
  std::string device_path_;
  int speed_;
  int flow_ctrl_;
  int databits_;
  int stopbits_;
  int parity_;
};

}  // namespace fyt::serial_driver

#endif  // SERIAL_DRIVER_UART_TRANSPORTER_HPP_
