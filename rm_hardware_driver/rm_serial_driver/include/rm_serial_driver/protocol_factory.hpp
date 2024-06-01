// Created by Chengfu Zou on 2023.7.6
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

#ifndef SERIAL_DRIVER_PROTOCOL_FACTORY_HPP_
#define SERIAL_DRIVER_PROTOCOL_FACTORY_HPP_

#include <memory>
#include <string_view>

#include "rm_serial_driver/protocol.hpp"
#include "rm_serial_driver/protocol/default_protocol.hpp"
#include "rm_serial_driver/protocol/infantry_protocol.hpp"
#include "rm_serial_driver/protocol/sentry_protocol.hpp"

namespace fyt::serial_driver {

class ProtocolFactory {
public:
  ProtocolFactory() = delete;
  // Factory method to create a protocol
  static std::unique_ptr<protocol::Protocol> createProtocol(std::string_view protocol_type,
                                                            std::string_view port_name,
                                                            bool enable_data_print) {
    if (protocol_type == "infantry") {
      return std::make_unique<protocol::ProtocolInfantry>(port_name, enable_data_print);
    }
    if (protocol_type == "hero") {
      return std::make_unique<protocol::DefaultProtocol>(port_name, enable_data_print);
    }
    if (protocol_type == "air") {
      return std::make_unique<protocol::DefaultProtocol>(port_name, enable_data_print);
    }
    if (protocol_type == "sentry") {
      return std::make_unique<protocol::ProtocolSentry>(port_name, enable_data_print);
    }

    return nullptr;
  }
};

};      // namespace fyt::serial_driver
#endif  // SERIAL_DRIVER_PROTOCOL_FACTORY_HPP_
