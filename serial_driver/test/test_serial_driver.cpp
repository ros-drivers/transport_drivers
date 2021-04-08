// Copyright 2021 LeoDrive, Copyright 2021 The Autoware Foundation
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

#include <gtest/gtest.h>

#include <string>

#include "serial_driver/serial_driver.hpp"

using drivers::serial_driver::FlowControl;
using drivers::serial_driver::Parity;
using drivers::serial_driver::SerialDriver;
using drivers::serial_driver::SerialPortConfig;
using drivers::serial_driver::StopBits;

static constexpr const char * dev_name = "/dev/ttyS0";
static constexpr uint32_t baud = 115200;
static constexpr FlowControl fc = FlowControl::NONE;
static constexpr Parity pt = Parity::NONE;
static constexpr StopBits sb = StopBits::ONE;

TEST(SerialDriverTest, PropertiesTest)
{
  IoContext ctx;
  SerialPortConfig config(baud, fc, pt, sb);
  SerialDriver driver(ctx);

  EXPECT_EQ(driver.port().get(), nullptr);

  driver.init_port(dev_name, config);

  EXPECT_EQ(driver.port()->device_name(), dev_name);

  ctx.waitForExit();
}
