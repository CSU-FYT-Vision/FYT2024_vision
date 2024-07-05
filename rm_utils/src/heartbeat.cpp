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

#include "rm_utils/heartbeat.hpp"

namespace fyt {
HeartBeatPublisher::SharedPtr HeartBeatPublisher::create(rclcpp::Node *node) {
  return std::shared_ptr<HeartBeatPublisher>(new HeartBeatPublisher(node));
}

HeartBeatPublisher::HeartBeatPublisher(rclcpp::Node *node) {
  // Initialize message
  message_.data = 0;
  // Create publisher
  std::string node_name = node->get_name();
  std::string topic_name = node_name + "/heartbeat";
  publisher_ = node->create_publisher<std_msgs::msg::Int64>(topic_name, 1);
  // Start publishing thread
  pub_thread_ = std::thread([this]() {
    while (rclcpp::ok()) {
      message_.data++;
      publisher_->publish(message_);
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }
  });
}

HeartBeatPublisher::~HeartBeatPublisher() {
  if (pub_thread_.joinable()) {
    pub_thread_.join();
  }
}

}  // namespace fyt
