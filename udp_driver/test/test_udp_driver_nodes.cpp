// Copyright 2021 LeoDrive.
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

// Developed by LeoDrive, 2021

#include <gtest/gtest.h>

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include <lifecycle_msgs/msg/state.hpp>
#include <rclcpp/rclcpp.hpp>

#include "udp_driver/udp_receiver_node.hpp"
#include "udp_driver/udp_sender_node.hpp"

using drivers::common::IoContext;
using drivers::udp_driver::UdpSocket;
using drivers::udp_driver::UdpReceiverNode;
using drivers::udp_driver::UdpSenderNode;
using lifecycle_msgs::msg::State;

const char ip[] = "127.0.0.1";
constexpr uint16_t port = 8000;

TEST(UdpSenderNodeTest, RosMessageToRawUdpMessageSharedContext)
{
  rclcpp::init(0, nullptr);
  IoContext ctx{};

  uint16_t sum = 0;
  std::promise<bool> promise_1;
  std::shared_future<bool> future_1(promise_1.get_future());

  // Raw UDP packets receiver, It could be a hardware (microcontroller, etc.)
  UdpSocket receiver(ctx, ip, port);
  receiver.open();
  EXPECT_EQ(receiver.isOpen(), true);
  sum = 0;
  receiver.bind();
  receiver.asyncReceive(
    [&](const std::vector<uint8_t> & buffer) {
      // Receive stream => 0 + 1 + 2+ 3 + 4 + 5 + 6 + 7 + 8 + 9 = 45
      sum = 0;
      // convert buffer to std::vector<uint8_t>
      for (int i = 0; i < static_cast<int>(buffer.size()); i += 2) {
        sum += *reinterpret_cast<const uint16_t *>(&buffer[i]);
      }
      if (sum == 45) {
        promise_1.set_value(true);
      }
    });

  // Main UDP driver node
  // The data this node receives will be pumped to device by raw UDP packets.
  rclcpp::NodeOptions options;
  options.append_parameter_override("ip", ip);
  options.append_parameter_override("port", port);
  auto node = std::make_shared<UdpSenderNode>(options, ctx);

  // Transition the node to active
  EXPECT_EQ(node->configure().id(), State::PRIMARY_STATE_INACTIVE);
  EXPECT_EQ(node->activate().id(), State::PRIMARY_STATE_ACTIVE);

  // ROS node publisher that sends sequence of data in ROS messages to device
  // The node sends data by 10 times and then shutting down
  // The data this node send will be received by main driver node and then pumped to device by raw
  // UDP packets.
  {
    // Send stream => 0 + 1 + 2+ 3 + 4 + 5 + 6 + 7 + 8 + 9 = 45
    std::promise<bool> promise_2;
    std::shared_future<bool> future_2(promise_2.get_future());

    auto minimal_publisher = std::make_shared<rclcpp::Node>("minimal_publisher");
    auto pub = minimal_publisher->create_publisher<udp_msgs::msg::UdpPacket>(
      "udp_write", 100);

    uint16_t count = 0;

    auto timer = minimal_publisher->create_wall_timer(
      std::chrono::milliseconds(10),
      [&](rclcpp::TimerBase & timer) {
        udp_msgs::msg::UdpPacket message;
        // push back each byte to the byte vector
        message.data.resize((count + 1) * 2);
        for (int i = 0; i <= count; i++) {
          std::memcpy(&message.data[i * 2], &i, sizeof(i));
        }
        pub->publish(message);
        if (count++ == 9) {
          timer.cancel();
          promise_2.set_value(true);
        }
      });

    ASSERT_EQ(
      rclcpp::spin_until_future_complete(
        minimal_publisher, future_2,
        std::chrono::milliseconds(4000)),
      rclcpp::FutureReturnCode::SUCCESS);
    EXPECT_EQ(future_2.get(), true);
  }

  ASSERT_EQ(
    rclcpp::spin_until_future_complete(
      node,
      future_1,
      std::chrono::milliseconds(4000)),
    rclcpp::FutureReturnCode::SUCCESS);
  EXPECT_EQ(future_1.get(), true);
  EXPECT_EQ(sum, 45);

  receiver.close();
  EXPECT_EQ(receiver.isOpen(), false);

  // Transition the node to finalized
  EXPECT_EQ(node->deactivate().id(), State::PRIMARY_STATE_INACTIVE);
  EXPECT_EQ(node->shutdown().id(), State::PRIMARY_STATE_FINALIZED);

  rclcpp::shutdown();

  ctx.waitForExit();
}

TEST(UdpReceiverNodeTest, RawUdpMessageToRosMessageSharedContext)
{
  rclcpp::init(0, nullptr);
  IoContext ctx{};

  int32_t sum = 0;

  // Main Drive Node
  rclcpp::NodeOptions options;
  options.append_parameter_override("ip", ip);
  options.append_parameter_override("port", port);
  auto node = std::make_shared<UdpReceiverNode>(options, ctx);

  // Transition the node to active
  EXPECT_EQ(node->configure().id(), State::PRIMARY_STATE_INACTIVE);
  EXPECT_EQ(node->activate().id(), State::PRIMARY_STATE_ACTIVE);

  // Receive stream => 0 + 1 + 2+ 3 + 4 + 5 + 6 + 7 + 8 + 9 = 45
  // Test node that receives sequence of data from a topic published by main node
  // computes the sum of sequence data and shutdowns when the sequence sum reaches to 45.
  std::promise<bool> promise_2;
  std::shared_future<bool> future_2(promise_2.get_future());

  auto minimal_subscriber = std::make_shared<rclcpp::Node>("minimal_subscriber");
  auto callback =
    [&](udp_msgs::msg::UdpPacket::SharedPtr msg) {
      sum = 0;
      // convert buffer to std::vector<uint8_t>
      std::cout << "msg size: " << msg->data.size() << std::endl;
      for (int i = 0; i < static_cast<int>(msg->data.size()); i += 4) {
        sum += *reinterpret_cast<const int32_t *>(&msg->data[i]);
        std::cout << "sum: " << sum << std::endl;
      }

      if (sum == 45) {
        promise_2.set_value(true);
      }
    };
  auto sub = minimal_subscriber->create_subscription<udp_msgs::msg::UdpPacket>(
    "udp_read",
    10,
    callback);

  // Sender socket that could be a hardware (microcontroller, etc.)
  // Streams sequence of data in an asynchronous manner by 10 times and then shutting down
  {
    UdpSocket sender(ctx, ip, port);
    sender.open();
    EXPECT_EQ(sender.isOpen(), true);
    int32_t count = 0;
    while (count <= 9) {
      std::vector<uint8_t> buffer;
      buffer.resize((count + 1) * 4);  // resize vector to length of bytes
      for (int32_t i = 0; i <= count; i++) {
        std::memcpy(&buffer[i * 4], &i, sizeof(i));
      }
      sender.asyncSend(buffer);
      count++;
    }
    EXPECT_EQ(count, 10);
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    sender.close();
    EXPECT_EQ(sender.isOpen(), false);
  }

  ASSERT_EQ(
    rclcpp::spin_until_future_complete(
      minimal_subscriber,
      future_2,
      std::chrono::milliseconds(2000)),
    rclcpp::FutureReturnCode::SUCCESS);
  EXPECT_EQ(future_2.get(), true);
  EXPECT_EQ(sum, 45);

  // Transition the node to finalized
  EXPECT_EQ(node->deactivate().id(), State::PRIMARY_STATE_INACTIVE);
  EXPECT_EQ(node->shutdown().id(), State::PRIMARY_STATE_FINALIZED);

  rclcpp::shutdown();

  ctx.waitForExit();
}
