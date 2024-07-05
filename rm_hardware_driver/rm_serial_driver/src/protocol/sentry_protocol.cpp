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

#include "rm_serial_driver/protocol/sentry_protocol.hpp"
// ros2
#include <geometry_msgs/msg/twist.hpp>

namespace fyt::serial_driver::protocol {
ProtocolSentry::ProtocolSentry(std::string_view port_name, bool enable_data_print) {
  auto uart_transporter = std::make_shared<UartTransporter>(std::string(port_name));
  packet_tool_ = std::make_shared<FixedPacketTool<32>>(uart_transporter);
  packet_tool_->enbaleDataPrint(enable_data_print);
}

void ProtocolSentry::send(const rm_interfaces::msg::GimbalCmd &data) {
  packet_.loadData<unsigned char>(data.fire_advice ? FireState::Fire : FireState::NotFire, 1);
  // is_spin
  // packet_.loadData<unsigned char>(0x00, 2);
  // gimbal control
  packet_.loadData<float>(static_cast<float>(data.pitch), 4);
  packet_.loadData<float>(static_cast<float>(data.yaw), 8);
  packet_.loadData<float>(static_cast<float>(data.distance), 12);
  // // chassis control
  // // linear x
  // packet_.loadData<float>(0, 16);
  // // linear y
  // packet_.loadData<float>(0, 20);
  // // angular z
  // packet_.loadData<float>(0, 24);
  // // useless data
  // packet_.loadData<float>(0, 28);
  packet_tool_->sendPacket(packet_);
}

void ProtocolSentry::send(const rm_interfaces::msg::ChassisCmd &data) {
  // packet_.loadData<unsigned char>(0x00, 1);
  // is_spin
  packet_.loadData<unsigned char>(data.is_spining ? 0x01 : 0x00, 2);
  packet_.loadData<unsigned char>(data.is_navigating ? 0x01 : 0x00, 3);
  // gimbal control
  // packet_.loadData<float>(0, 4);
  // packet_.loadData<float>(0, 8);
  // packet_.loadData<float>(0, 12);
  // chassis control
  // linear x
  packet_.loadData<float>(data.twist.linear.x, 16);
  // linear y
  packet_.loadData<float>(data.twist.linear.y, 20);
  // angular z
  packet_.loadData<float>(data.twist.angular.z, 24);
  // useless data
  // packet_.loadData<float>(0, 28);
  packet_tool_->sendPacket(packet_);
}

bool ProtocolSentry::receive(rm_interfaces::msg::SerialReceiveData &data) {
  FixedPacket<32> packet;
  if (packet_tool_->recvPacket(packet)) {
     // game status
    uint8_t enemy_color;
    packet.unloadData(enemy_color, 1);
    data.mode = (enemy_color == ENEMY_BLUE ? 1 : 0);

    packet.unloadData(data.pitch, 2);
    packet.unloadData(data.yaw, 6);
    // 实际上是底盘角度
    // packet.unloadData(data.chassis_yaw, 10);
    // blood
    packet.unloadData(data.judge_system_data.blood, 14);
    // remaining time
    packet.unloadData(data.judge_system_data.remaining_time, 16);
    // outpost hp
    packet.unloadData(data.judge_system_data.outpost_hp, 20);
    // operator control message
    packet.unloadData(data.judge_system_data.operator_command.is_outpost_attacking, 22);
    packet.unloadData(data.judge_system_data.operator_command.is_retreating, 23);
    packet.unloadData(data.judge_system_data.operator_command.is_drone_avoiding, 24);

    packet.unloadData(data.judge_system_data.game_status, 25);

    data.bullet_speed = 25;
    return true;
  } else {
    return false;
  }
}

std::vector<rclcpp::SubscriptionBase::SharedPtr> ProtocolSentry::getSubscriptions(
  rclcpp::Node::SharedPtr node) {
  auto sub1 = node->create_subscription<rm_interfaces::msg::GimbalCmd>(
    "armor_solver/cmd_gimbal",
    rclcpp::SensorDataQoS(),
    [this](const rm_interfaces::msg::GimbalCmd::SharedPtr msg) { this->send(*msg); });
  auto sub2 = node->create_subscription<rm_interfaces::msg::GimbalCmd>(
    "rune_solver/cmd_gimbal",
    rclcpp::SensorDataQoS(),
    [this](const rm_interfaces::msg::GimbalCmd::SharedPtr msg) { this->send(*msg); });
  auto sub3 = node->create_subscription<rm_interfaces::msg::ChassisCmd>(
    "/cmd_chassis",
    rclcpp::SensorDataQoS(),
    [this](const rm_interfaces::msg::ChassisCmd::SharedPtr msg) { this->send(*msg); });
  return {sub1, sub2, sub3};
}

std::vector<rclcpp::Client<rm_interfaces::srv::SetMode>::SharedPtr> ProtocolSentry::getClients(
  rclcpp::Node::SharedPtr node) const {
  auto client1 = node->create_client<rm_interfaces::srv::SetMode>("armor_detector/set_mode",
                                                                  rmw_qos_profile_services_default);
  auto client2 = node->create_client<rm_interfaces::srv::SetMode>("armor_solver/set_mode",
                                                                  rmw_qos_profile_services_default);
  return {client1, client2};
}
}  // namespace fyt::serial_driver::protocol
