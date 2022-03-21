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

#include <thread>
#include <vector>

#include "udp_driver/udp_socket.hpp"

using drivers::common::IoContext;
using drivers::udp_driver::UdpSocket;

const char ip[] = "127.0.0.1";
constexpr uint16_t port = 8000;
static float PI = 3.14159265359;

void handle_data(const std::vector<uint8_t> & buffer)
{
  float received_PI = *reinterpret_cast<const float *>(&buffer[0]);
  EXPECT_EQ(buffer.size(), sizeof(received_PI));
  EXPECT_EQ(received_PI, PI);
}

TEST(UdpDataTest, LifeCycleTest)
{
  IoContext ctx;
  UdpSocket sender(ctx, ip, port);
  UdpSocket receiver(ctx, ip, port);

  EXPECT_EQ(sender.remote_ip(), ip);
  EXPECT_EQ(sender.remote_port(), port);
  EXPECT_EQ(receiver.remote_ip(), ip);
  EXPECT_EQ(receiver.remote_port(), port);

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

TEST(UdpDataTest, BlockingSendReceiveTest)
{
  IoContext ctx;
  UdpSocket sender(ctx, ip, port);
  UdpSocket receiver(ctx, ip, port);

  sender.open();
  EXPECT_EQ(sender.isOpen(), true);
  receiver.open();
  EXPECT_EQ(receiver.isOpen(), true);

  receiver.bind();

  std::vector<uint8_t> vector_to_send;
  vector_to_send.resize(sizeof(PI));
  std::memcpy(&vector_to_send[0], &PI, sizeof(PI));
  std::size_t size = sender.send(vector_to_send);
  EXPECT_EQ(size, sizeof(PI));

  float received_PI = 0.0f;
  std::vector<uint8_t> vector_to_fill;
  vector_to_fill.resize(sizeof(received_PI));
  size = receiver.receive(vector_to_fill);
  std::cout << "vector to fill: " << vector_to_fill.size() << std::endl;
  received_PI = *reinterpret_cast<float *>(&vector_to_fill[0]);
  EXPECT_EQ(size, sizeof(received_PI));
  EXPECT_EQ(received_PI, PI);

  sender.close();
  EXPECT_EQ(sender.isOpen(), false);
  receiver.close();
  EXPECT_EQ(receiver.isOpen(), false);
}

TEST(UdpDataTest, NonBlockingSendReceiveTest)
{
  IoContext ctx(8);
  EXPECT_EQ(ctx.serviceThreadCount(), uint32_t(8));

  UdpSocket sender(ctx, ip, port);
  UdpSocket receiver(ctx, ip, port);

  sender.open();
  EXPECT_EQ(sender.isOpen(), true);
  receiver.open();
  EXPECT_EQ(receiver.isOpen(), true);

  receiver.bind();
  receiver.asyncReceive(std::bind(handle_data, std::placeholders::_1));

  std::vector<uint8_t> buffer;
  buffer.resize(sizeof(PI));
  std::memcpy(buffer.data(), &PI, sizeof(PI));

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

TEST(UdpDataTest, BlockingSendNonBlockingReceiveTest)
{
  IoContext ctx(5);
  UdpSocket sender(ctx, ip, port);
  UdpSocket receiver(ctx, ip, port);

  sender.open();
  EXPECT_EQ(sender.isOpen(), true);
  receiver.open();
  EXPECT_EQ(receiver.isOpen(), true);

  receiver.bind();
  receiver.asyncReceive(std::bind(handle_data, std::placeholders::_1));

  std::vector<uint8_t> vector_to_send;
  vector_to_send.resize(sizeof(PI));
  std::memcpy(&vector_to_send[0], &PI, sizeof(PI));

  std::size_t size = sender.send(vector_to_send);
  EXPECT_EQ(size, sizeof(PI));

  std::this_thread::sleep_for(std::chrono::milliseconds(1000));

  sender.close();
  EXPECT_EQ(sender.isOpen(), false);
  receiver.close();
  EXPECT_EQ(receiver.isOpen(), false);

  ctx.waitForExit();
}
