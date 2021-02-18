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

#include "gtest/gtest.h"
#include "udp_driver/udp_socket.hpp"

using autoware::drivers::IoContext;
using autoware::drivers::udp_driver::UdpSocket;

const char ip[] = "127.0.0.1";
constexpr uint16_t port = 8000;
constexpr float PI = 3.14159265359;

void handle_data(const MutSocketBuffer & buffer)
{
  float received_PI = *reinterpret_cast<float *>(buffer.data());
  EXPECT_EQ(buffer.size(), sizeof(received_PI));
  EXPECT_EQ(received_PI, PI);
  std::cout << "[handle_data]" << std::endl;
}

TEST(UdpDataTest, LifeCycleTest) {
  IoContext ctx;
  UdpSocket sender(ctx, ip, port);
  UdpSocket receiver(ctx, ip, port);

  EXPECT_EQ(sender.ip(), ip);
  EXPECT_EQ(sender.port(), port);
  EXPECT_EQ(receiver.ip(), ip);
  EXPECT_EQ(receiver.port(), port);

  EXPECT_EQ(sender.isOpen(), false);
  sender.open();
  EXPECT_EQ(sender.isOpen(), true);

  EXPECT_EQ(receiver.isOpen(), false);
  receiver.open();
  EXPECT_EQ(receiver.isOpen(), true);

  sender.close();
  EXPECT_EQ(sender.isOpen(), false);
  receiver.close();
  EXPECT_EQ(receiver.isOpen(), false);
}

TEST(UdpDataTest, BlockingSendReceiveTest) {
  IoContext ctx;
  UdpSocket sender(ctx, ip, port);
  UdpSocket receiver(ctx, ip, port);

  sender.open();
  EXPECT_EQ(sender.isOpen(), true);
  receiver.open();
  EXPECT_EQ(receiver.isOpen(), true);

  receiver.bind();

  std::size_t size = sender.send(
    MutSocketBuffer(reinterpret_cast<void *>(const_cast<float *>(&PI)), sizeof(PI)));
  EXPECT_EQ(size, sizeof(PI));

  float received_PI = 0.0f;
  size = receiver.receive(MutSocketBuffer(&received_PI, sizeof(received_PI)));
  EXPECT_EQ(size, sizeof(received_PI));
  EXPECT_EQ(received_PI, PI);

  sender.close();
  EXPECT_EQ(sender.isOpen(), false);
  receiver.close();
  EXPECT_EQ(receiver.isOpen(), false);
}

TEST(UdpDataTest, NonBlockingSendReceiveTest) {
  IoContext ctx(8);
  EXPECT_EQ(ctx.serviceThreadCount(), 8);

  UdpSocket sender(ctx, ip, port);
  UdpSocket receiver(ctx, ip, port);

  sender.open();
  EXPECT_EQ(sender.isOpen(), true);
  receiver.open();
  EXPECT_EQ(receiver.isOpen(), true);

  receiver.bind();
  receiver.asyncReceive(boost::bind(handle_data, _1));

  MutSocketBuffer buffer(reinterpret_cast<void *>(const_cast<float *>(&PI)), sizeof(PI));
  sender.asyncSend(buffer);
  sender.asyncSend(buffer);
  sender.asyncSend(buffer);

  std::this_thread::sleep_for(std::chrono::milliseconds(1000));

  sender.close();
  EXPECT_EQ(sender.isOpen(), false);
  receiver.close();
  EXPECT_EQ(receiver.isOpen(), false);

  ctx.waitForExit();
}

TEST(UdpDataTest, BlockingSendNonBlockingReceiveTest) {
  IoContext ctx(5);
  UdpSocket sender(ctx, ip, port);
  UdpSocket receiver(ctx, ip, port);

  sender.open();
  EXPECT_EQ(sender.isOpen(), true);
  receiver.open();
  EXPECT_EQ(receiver.isOpen(), true);

  receiver.bind();
  receiver.asyncReceive(boost::bind(handle_data, _1));

  std::size_t size = sender.send(
    MutSocketBuffer(reinterpret_cast<void *>(const_cast<float *>(&PI)), sizeof(PI)));
  EXPECT_EQ(size, sizeof(PI));

  std::this_thread::sleep_for(std::chrono::milliseconds(1000));

  sender.close();
  EXPECT_EQ(sender.isOpen(), false);
  receiver.close();
  EXPECT_EQ(receiver.isOpen(), false);

  ctx.waitForExit();
}
