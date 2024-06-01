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

#include "rm_serial_driver/uart_transporter.hpp"
// System
#include <errno.h>  /*错误号定义*/
#include <fcntl.h>  /*文件控制定义*/
#include <stdio.h>  /*标准输入输出定义*/
#include <stdlib.h> /*标准函数库定义*/
#include <string.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <termios.h> /*PPSIX 终端控制定义*/
#include <unistd.h>  /*Unix 标准函数定义*/

namespace fyt::serial_driver {

bool UartTransporter::setParam(int speed, int flow_ctrl, int databits, int stopbits, int parity) {
  // 设置串口数据帧格式
  int speed_arr[] = {B115200, B19200, B9600, B4800, B2400, B1200, B300};
  int name_arr[] = {115200, 19200, 9600, 4800, 2400, 1200, 300};
  struct termios options;
  // tcgetattr(fd,&options)得到与fd指向对象的相关参数，并将它们保存于options,该函数还可以测试配置是否正确，
  // 该串口是否可用等。若调用成功，函数返回值为0，若调用失败，函数返回值为1.
  if (tcgetattr(fd_, &options) != 0) {
    error_message_ = "Setup Serial err";
    return false;
  }
  // 设置串口输入波特率和输出波特率
  for (size_t i = 0; i < sizeof(speed_arr) / sizeof(int); i++) {
    if (speed == name_arr[i]) {
      cfsetispeed(&options, speed_arr[i]);
      cfsetospeed(&options, speed_arr[i]);
    }
  }
  // 修改控制模式，保证程序不会占用串口
  options.c_cflag |= CLOCAL;
  // 修改控制模式，使得能够从串口中读取输入数据
  options.c_cflag |= CREAD;
  // 设置数据流控制
  switch (flow_ctrl) {
    case 0:  // 不使用流控制
      options.c_cflag &= ~CRTSCTS;
      break;
    case 1:  // 使用硬件流控制
      options.c_cflag |= CRTSCTS;
      break;
    case 2:  // 使用软件流控制
      options.c_cflag |= IXON | IXOFF | IXANY;
      break;
  }
  // 设置数据位
  // 屏蔽其他标志位
  options.c_cflag &= ~CSIZE;
  switch (databits) {
    case 5:
      options.c_cflag |= CS5;
      break;
    case 6:
      options.c_cflag |= CS6;
      break;
    case 7:
      options.c_cflag |= CS7;
      break;
    case 8:
      options.c_cflag |= CS8;
      break;
    default:
      error_message_ = "Unsupported data size";
      return false;
  }
  // 设置校验位
  switch (parity) {
    case 'n':
    case 'N':  // 无奇偶校验位。
      options.c_cflag &= ~PARENB;
      options.c_iflag &= ~INPCK;
      break;
    case 'o':
    case 'O':  // 设置为奇校验
      options.c_cflag |= (PARODD | PARENB);
      options.c_iflag |= INPCK;
      break;
    case 'e':
    case 'E':  // 设置为偶校验
      options.c_cflag |= PARENB;
      options.c_cflag &= ~PARODD;
      options.c_iflag |= INPCK;
      break;
    case 's':
    case 'S':  // 设置为空格
      options.c_cflag &= ~PARENB;
      options.c_cflag &= ~CSTOPB;
      break;
    default:
      error_message_ = "Unsupported parity";
      return false;
  }
  // 设置停止位
  switch (stopbits) {
    case 1:
      options.c_cflag &= ~CSTOPB;
      break;
    case 2:
      options.c_cflag |= CSTOPB;
      break;
    default:
      error_message_ = "Unsupported stop bits";
      return false;
  }

  // 修改输出模式，原始数据输出
  options.c_oflag &= ~OPOST;
  options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
  // 传输特殊字符，否则特殊字符0x0d,0x11,0x13会被屏蔽或映射。
  options.c_iflag &= ~(BRKINT | ICRNL | INPCK | ISTRIP | IXON);

  // 设置等待时间和最小接收字符
  options.c_cc[VTIME] = 1;  // 读取一个字符等待1*(1/10)s
  options.c_cc[VMIN] = 1;   // 读取字符的最少个数为1
  tcflush(fd_, TCIFLUSH);

  // 激活配置 (将修改后的termios数据设置到串口中）
  if (tcsetattr(fd_, TCSANOW, &options) != 0) {
    error_message_ = "com set error";
    return false;
  }
  return true;
}

bool UartTransporter::open() {
  if (is_open_) {
    return true;
  }
  fd_ = ::open(device_path_.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
  if (-1 == fd_) {
    error_message_ = "can't open uart device: " + device_path_;
    return false;
  }
  // 恢复串口为阻塞状态
  if (fcntl(fd_, F_SETFL, 0) < 0) {
    error_message_ = "fcntl failed";
    return false;
  }
  // 测试是否为终端设备
  // 避免自启动无法读取数据
  // if (0 == isatty(STDIN_FILENO)) {
  //   error_message_ = "standard input is not a terminal device";
  //   return false;
  // }
  // 设置串口数据帧格式
  if (!setParam(speed_, flow_ctrl_, databits_, stopbits_, parity_)) {
    return false;
  }
  is_open_ = true;
  return true;
}

void UartTransporter::close() {
  if (!is_open_) {
    return;
  }
  ::close(fd_);
  fd_ = -1;
  is_open_ = false;
}

bool UartTransporter::isOpen() { return is_open_; }

int UartTransporter::read(void *buffer, size_t len) {
  int ret = ::read(fd_, buffer, len);
  // tcflush(fd_, TCIFLUSH);
  return ret;
}

int UartTransporter::write(const void *buffer, size_t len) {
  int ret = ::write(fd_, buffer, len);
  return ret;
}

}  // namespace fyt::serial_driver
