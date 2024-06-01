// Copyright 2021 RoboMaster-OSS
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

#include <memory>
#include <string>
#include <vector>

#include "dummy_transporter.hpp"
#include "gtest/gtest.h"
#include "rm_serial_driver/fixed_packet.hpp"
#include "rm_serial_driver/fixed_packet_tool.hpp"

using namespace fyt;
TEST(FixedPacketTool, construct_with_nullptr) {
  EXPECT_THROW(serial_driver::FixedPacketTool<32>(nullptr), std::invalid_argument);
}

TEST(FixedPacketTool, send_and_recv) {
  auto factory = std::make_shared<TransporterFactory>();
  auto transporter1 = factory->get_transporter1();
  auto transporter2 = factory->get_transporter2();
  auto packet_tool1 = std::make_shared<serial_driver::FixedPacketTool<32>>(transporter1);
  auto packet_tool2 = std::make_shared<serial_driver::FixedPacketTool<32>>(transporter2);
  serial_driver::FixedPacket<32> packet1, packet2;
  // send
  int a = 10;
  packet1.loadData(a, 10);
  bool send_ret = packet_tool1->sendPacket(packet1);
  ASSERT_TRUE(send_ret);
  // recv
  int b;
  bool recv_ret = packet_tool2->recvPacket(packet2);
  ASSERT_TRUE(recv_ret);
  packet2.unloadData<int>(b, 10);
  EXPECT_EQ(a, b);
}

TEST(FixedPacketTool, realtime_send) {
  auto factory = std::make_shared<TransporterFactory>();
  auto transporter1 = factory->get_transporter1();
  auto transporter2 = factory->get_transporter2();
  auto packet_tool1 = std::make_shared<serial_driver::FixedPacketTool<32>>(transporter1);
  auto packet_tool2 = std::make_shared<serial_driver::FixedPacketTool<32>>(transporter2);
  packet_tool1->enbaleRealtimeSend(true);
  serial_driver::FixedPacket<32> packet1, packet2;
  // recv
  auto t = std::thread([&]() {
    int b;
    for (int i = 0; i < 10; i++) {
      bool recv_ret = packet_tool2->recvPacket(packet2);
      ASSERT_TRUE(recv_ret);
      packet2.unloadData<int>(b, 10);
      EXPECT_EQ(i, b);
    }
  });
  // send
  for (int i = 0; i < 10; i++) {
    packet1.loadData(i, 10);
    bool send_ret = packet_tool1->sendPacket(packet1);
    ASSERT_TRUE(send_ret);
  }
  t.join();
}