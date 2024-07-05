// Created by Chengfu Zou
// Copyright (C) FYT Vision Group. All rights reserved.

// std
#include <chrono>
#include <condition_variable>
#include <future>
#include <memory>
#include <opencv2/calib3d.hpp>
#include <rclcpp/executors.hpp>
#include <thread>
// ros2
#include <tf2_ros/transform_broadcaster.h>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/node_options.hpp>
#include <rclcpp/rclcpp.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
// project
#include "rm_interfaces/msg/serial_receive_data.hpp"
#include "rm_interfaces/srv/set_mode.hpp"
#include "rm_utils/logger/log.hpp"
#include "rm_utils/math/utils.hpp"
#include "rm_utils/heartbeat.hpp"

namespace fyt::serial_driver {
class VirtualSerialNode : public rclcpp::Node {
  struct SetModeClient {
    SetModeClient(rclcpp::Client<rm_interfaces::srv::SetMode>::SharedPtr p) : ptr(p) {}
    std::atomic<bool> on_waiting = false;
    std::atomic<int> mode = 0;
    rclcpp::Client<rm_interfaces::srv::SetMode>::SharedPtr ptr;
  };

public:
  explicit VirtualSerialNode(const rclcpp::NodeOptions &options) : Node("serial_driver", options) {
    FYT_REGISTER_LOGGER("serial_driver", "~/fyt2024-log", INFO);
    FYT_INFO("serial_driver", "Starting VirtualSerialNode!");

    serial_receive_data_pub_ =
      this->create_publisher<rm_interfaces::msg::SerialReceiveData>("serial/receive", 10);

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    this->declare_parameter("vision_mode", static_cast<int>(0));

    has_rune_ = this->declare_parameter("has_rune", true);

    serial_receive_data_msg_.header.frame_id = "odom";
    serial_receive_data_msg_.bullet_speed = 25.0;
    transform_stamped_.header.frame_id = "odom";
    transform_stamped_.child_frame_id = "gimbal_link";
    serial_receive_data_msg_.mode = 0;
    serial_receive_data_msg_.roll = this->declare_parameter("roll", 0.0);
    serial_receive_data_msg_.pitch = this->declare_parameter("pitch", 0.0);
    serial_receive_data_msg_.yaw = this->declare_parameter("yaw", 0.0);

    // Heartbeat
    heartbeat_ = HeartBeatPublisher::create(this);

    // Param client
    auto autoaim_set_mode_client_1 =
      this->create_client<rm_interfaces::srv::SetMode>("armor_detector/set_mode");
    set_mode_clients_.emplace(autoaim_set_mode_client_1->get_service_name(),
                              autoaim_set_mode_client_1);
    auto autoaim_set_mode_client_2 =
      this->create_client<rm_interfaces::srv::SetMode>("armor_solver/set_mode");
    set_mode_clients_.emplace(autoaim_set_mode_client_2->get_service_name(),
                              autoaim_set_mode_client_2);
    if (has_rune_) {
      auto client1 = this->create_client<rm_interfaces::srv::SetMode>("rune_detector/set_mode");
      set_mode_clients_.emplace(client1->get_service_name(), client1);
      auto client2 = this->create_client<rm_interfaces::srv::SetMode>("rune_solver/set_mode");
      set_mode_clients_.emplace(client2->get_service_name(), client2);
    }

    timer_ = this->create_wall_timer(std::chrono::milliseconds(1), [this]() {
      serial_receive_data_msg_.header.stamp = this->now();
      int mode = this->get_parameter("vision_mode").as_int();
      double roll = this->get_parameter("roll").as_double();
      double pitch = this->get_parameter("pitch").as_double();
      double yaw = this->get_parameter("yaw").as_double();
      serial_receive_data_msg_.mode = mode;
      serial_receive_data_msg_.pitch = pitch;
      serial_receive_data_msg_.yaw = yaw;
      tf2::Quaternion q;
      q.setRPY(roll * M_PI / 180.0, -pitch * M_PI / 180.0, yaw * M_PI / 180.0);
      transform_stamped_.transform.rotation = tf2::toMsg(q);
      transform_stamped_.header.frame_id = "odom";
      transform_stamped_.child_frame_id = "gimbal_link";
      // serial_receive_data_msg.mode = mode;
      serial_receive_data_pub_->publish(serial_receive_data_msg_);
      transform_stamped_.header.stamp = this->now();
      tf_broadcaster_->sendTransform(transform_stamped_);

      Eigen::Quaterniond q_eigen(q.w(), q.x(), q.y(), q.z());
      Eigen::Vector3d rpy  = utils::getRPY(q_eigen.toRotationMatrix());
      q.setRPY(rpy[0], 0, 0);
      transform_stamped_.transform.rotation = tf2::toMsg(q);
      transform_stamped_.header.frame_id = "odom";
      transform_stamped_.child_frame_id = "odom_rectify";
      tf_broadcaster_->sendTransform(transform_stamped_);

      for (auto &[service_name, client] : set_mode_clients_) {
        if (client.mode.load() != mode && !client.on_waiting.load()) {
          setMode(client, mode);
        }
      }
    });
  }

  void setMode(SetModeClient &client, const uint8_t mode) {
    using namespace std::chrono_literals;

    std::string service_name = client.ptr->get_service_name();
    // Wait for service
    while (!client.ptr->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        FYT_ERROR(
          "serial_driver", "Interrupted while waiting for the service {}. Exiting.", service_name);
        return;
      }
      FYT_INFO("serial_driver", "service {} not available, waiting again...", service_name);
    }
    if (!client.ptr->service_is_ready()) {
      FYT_WARN("serial_driver", "Service: {} is not available!", service_name);
      return;
    }
    // Send request
    auto req = std::make_shared<rm_interfaces::srv::SetMode::Request>();
    req->mode = mode;
    client.on_waiting.store(true);
    auto result = client.ptr->async_send_request(
      req, [mode, &client](rclcpp::Client<rm_interfaces::srv::SetMode>::SharedFuture result) {
        client.on_waiting.store(false);
        if (result.get()->success) {
          client.mode.store(mode);
        }
      });
  }

private:
  HeartBeatPublisher::SharedPtr heartbeat_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Publisher<rm_interfaces::msg::SerialReceiveData>::SharedPtr serial_receive_data_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rm_interfaces::msg::SerialReceiveData serial_receive_data_msg_;
  geometry_msgs::msg::TransformStamped transform_stamped_;

  bool has_rune_;

  std::unordered_map<std::string, SetModeClient> set_mode_clients_;
};
}  // namespace fyt::serial_driver

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(fyt::serial_driver::VirtualSerialNode)
