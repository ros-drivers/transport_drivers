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

#include "udp_driver/udp_socket.hpp"

using drivers::common::IoContext;
using drivers::udp_driver::UdpSocket;

const char ip[] = "127.0.0.1";
constexpr uint16_t port = 8000;

TEST(UdpSocketTest, LifeCycleTest)
{
  IoContext ctx;
  UdpSocket socket(ctx, ip, port);

  EXPECT_EQ(socket.remote_ip(), ip);
  EXPECT_EQ(socket.remote_port(), port);

  EXPECT_EQ(socket.isOpen(), false);
  socket.open();
  EXPECT_EQ(socket.isOpen(), true);
  socket.close();
  EXPECT_EQ(socket.isOpen(), false);
}
