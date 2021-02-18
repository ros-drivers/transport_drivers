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

#include <string>

#include "gtest/gtest.h"
#include "udp_driver/udp_socket.hpp"

using autoware::drivers::IoContext;
using autoware::drivers::udp_driver::UdpSocket;

TEST(UdpSenderTest, LifeCycleTest) {
  std::string ip = "127.0.0.1";
  uint16_t port = 8000;

  IoContext ctx;
  UdpSocket sender(ctx, ip, port);

  EXPECT_EQ(sender.ip(), ip);
  EXPECT_EQ(sender.port(), port);

  EXPECT_EQ(sender.isOpen(), false);
  sender.open();
  EXPECT_EQ(sender.isOpen(), true);
  sender.close();
  EXPECT_EQ(sender.isOpen(), false);
}
