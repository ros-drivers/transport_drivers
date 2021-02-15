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

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "gtest/gtest.h"
#include "udp_driver/udp_driver_node.hpp"

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/int32.hpp"

using namespace std::chrono_literals;
using namespace autoware::drivers; // includes IoContext
using namespace autoware::drivers::udp_driver;

const std::string ip = "127.0.0.1";
constexpr uint16_t port = 8000;

TEST(UdpDriverTest, NonBlockingSendReceiveTest) {
    IoContext ctx;
    UdpDriver driver(ctx);

    driver.initialize_sender(ip, port);
    driver.initialize_receiver(ip, port);

    int32_t sum = 0;
    driver.receiver()->open();
    driver.receiver()->bind();
    driver.receiver()->asyncReceive([&](const MutSocketBuffer &buffer) {
        sum += *(int32_t *) buffer.data();
    });

    driver.sender()->open();
    EXPECT_EQ(driver.sender()->isOpen(), true);

    for (int val : {1, 2, 3, 4, 5}) {
        driver.sender()->asyncSend(MutSocketBuffer(&val, sizeof(val)));
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(1000));

    EXPECT_EQ(sum, 15);

    driver.sender()->close();
    EXPECT_EQ(driver.sender()->isOpen(), false);

    driver.receiver()->close();
    EXPECT_EQ(driver.receiver()->isOpen(), false);

    driver.context().waitForExit();
}

TEST(MinimalPublisher, FromRosMessageToRawUdpMessageTest) {
    rclcpp::init(0, nullptr);
    IoContext ctx;

    int32_t sum = 0;
    std::promise<bool> promise_1;
    std::shared_future<bool> future_1(promise_1.get_future());

    // Raw UDP packets receiver, It could be a hardware (microcontroller, etc.)
    UdpSocket receiver(ctx, "127.0.0.1", 9000);
    receiver.open();
    EXPECT_EQ(receiver.isOpen(), true);
    receiver.bind();
    receiver.asyncReceive([&](const MutSocketBuffer &buffer) {
        // Recevied data => 0 + 1 + 2+ 3 + 4 + 5 + 6 + 7 + 8 + 9 = 45
        sum += *(int32_t*)buffer.data();
        if (sum == 45) {
            promise_1.set_value(true);
        }
    });

    // Main UDP driver node
    // The data this node receives will be pumped to device by raw UDP packets.
    rclcpp::NodeOptions options;
    std::shared_ptr<UdpDriverNode> node(std::make_shared<UdpDriverNode>("UdpDriverNodeTest", options, ctx));
    node->initialize_sender("127.0.0.1", 9000);

    // ROS node publisher that sends sequence of data in ROS messages to device
    // The node sends data by 10 times and then shutting down
    // The data this node send will be received by main driver node and then pumped to device by raw
    // UDP packets.
    {
        // Send data => 0 + 1 + 2+ 3 + 4 + 5 + 6 + 7 + 8 + 9 = 45
        std::promise<bool> promise_2;
        std::shared_future<bool> future_2(promise_2.get_future());

        auto minimal_publisher = std::make_shared<rclcpp::Node>("minimal_publisher");
        auto pub = minimal_publisher->create_publisher<std_msgs::msg::Int32>("udp_write", 100);
        auto timer = minimal_publisher->create_wall_timer(10ms, [&](rclcpp::TimerBase & timer) {
            static int count = 0;
            std_msgs::msg::Int32 message;
            message.data = count;
            pub->publish(message);

            if (count++ == 9) {
                timer.cancel();
                promise_2.set_value(true);
            }
        });

        ASSERT_EQ(rclcpp::spin_until_future_complete(minimal_publisher, future_2, std::chrono::milliseconds(2000)),
                  rclcpp::FutureReturnCode::SUCCESS);
        EXPECT_EQ(future_2.get(), true);
    }

    ASSERT_EQ(rclcpp::spin_until_future_complete(node, future_1, std::chrono::milliseconds(2000)),
              rclcpp::FutureReturnCode::SUCCESS);
    EXPECT_EQ(future_1.get(), true);
    EXPECT_EQ(sum, 45);

    receiver.close();
    EXPECT_EQ(receiver.isOpen(), false);

    rclcpp::shutdown();
}

TEST(MinimalPublisher, FromRawUdpMessageToRosMessageTest) {
    rclcpp::init(0, nullptr);
    IoContext ctx;

    int32_t sum = 0;

    // Main Drive Node
    rclcpp::NodeOptions options;
    std::shared_ptr<UdpDriverNode> node(std::make_shared<UdpDriverNode>("UdpDriverNodeTest", options, ctx));
    node->initialize_receiver("127.0.0.1", 9000);

    // Receive data => 0 + 1 + 2+ 3 + 4 + 5 + 6 + 7 + 8 + 9 = 45
    // Test node that receives sequence of data from a topic published by main node
    // computes the sum of sequence data and shutdowns when the sequence sum reaches to 45.
    std::promise<bool> promise_2;
    std::shared_future<bool> future_2(promise_2.get_future());

    auto minimal_subscriber = std::make_shared<rclcpp::Node>("minimal_subscriber");
    auto sub = minimal_subscriber->create_subscription<std_msgs::msg::Int32>("udp_read",10,
     [&] (std_msgs::msg::Int32::SharedPtr msg) {
         sum += msg->data;
         if (sum == 45) {
             promise_2.set_value(true);
         }
     });

    // Sender socket that could be a hardware (microcontroller, etc.)
    // Streams sequence of data in an asynchronous manner by 10 times and then shutting down
    {
        UdpSocket sender(ctx, "127.0.0.1", 9000);
        sender.open();
        EXPECT_EQ(sender.isOpen(), true);
        int32_t count = 0;
        while (count <= 9) {
            MutSocketBuffer buffer((void *)&count, sizeof(count));
            sender.asyncSend(buffer);
            count++;
        }
        EXPECT_EQ(count, 10);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        sender.close();
        EXPECT_EQ(sender.isOpen(), false);
    }

    ASSERT_EQ(rclcpp::spin_until_future_complete(minimal_subscriber, future_2, std::chrono::milliseconds(2000)),
              rclcpp::FutureReturnCode::SUCCESS);
    EXPECT_EQ(future_2.get(), true);
    EXPECT_EQ(sum, 45);

    rclcpp::shutdown();
}
