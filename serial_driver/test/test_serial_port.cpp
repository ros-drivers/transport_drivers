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
#include <vector>

#include "serial_driver/serial_port.hpp"

using spb = asio::serial_port_base;
using drivers::serial_driver::FlowControl;
using drivers::serial_driver::Parity;
using drivers::serial_driver::SerialPort;
using drivers::serial_driver::SerialPortConfig;
using drivers::serial_driver::StopBits;

static constexpr const char * dev_name = "/dev/ttyS0";
static constexpr const char * dev_null = "/dev/null";
static constexpr uint32_t baud = 115200;
static constexpr FlowControl fc = FlowControl::NONE;
static constexpr Parity pt = Parity::NONE;
static constexpr StopBits sb = StopBits::ONE;

TEST(SerialPortTest, PropertiesTest)
{
  IoContext ctx;
  SerialPortConfig config(baud, fc, pt, sb);
  SerialPort port(ctx, dev_name, config);

  // Test internally-defined types
  EXPECT_EQ(port.device_name(), dev_name);
  EXPECT_EQ(port.serial_port_config().get_baud_rate(), baud);
  EXPECT_EQ(port.serial_port_config().get_flow_control(), fc);
  EXPECT_EQ(port.serial_port_config().get_parity(), pt);
  EXPECT_EQ(port.serial_port_config().get_stop_bits(), sb);

  // Test ASIO types
  EXPECT_EQ(port.serial_port_config().get_baud_rate_asio().value(), spb::baud_rate{baud}.value());
  EXPECT_EQ(port.serial_port_config().get_flow_control_asio(), spb::flow_control::none);
  EXPECT_EQ(port.serial_port_config().get_parity_asio(), spb::parity::none);
  EXPECT_EQ(port.serial_port_config().get_stop_bits_asio(), spb::stop_bits::one);

  ctx.waitForExit();
}

TEST(SerialPortTest, StateTest)
{
  IoContext ctx;
  SerialPortConfig config(baud, fc, pt, sb);
  SerialPort port(ctx, dev_name, config);

  std::vector<uint8_t> send_recv_buff;

  EXPECT_FALSE(port.is_open());
  EXPECT_THROW(port.send(send_recv_buff), asio::system_error);
  EXPECT_THROW(port.receive(send_recv_buff), asio::system_error);

  // Can't test other functions without available port

  ctx.waitForExit();
}

TEST(SerialDriverTest, SendBreakWhileClosed)
{
  IoContext ctx;
  SerialPortConfig config(baud, fc, pt, sb);
  SerialPort port(ctx, dev_null, config);

  EXPECT_FALSE(port.is_open());
  // Without the port open, expect this to fail
  EXPECT_FALSE(port.send_break());

  ctx.waitForExit();
}

TEST(SerialDriverTest, SendBreakWhenFailtToOpen)
{
  IoContext ctx;
  SerialPortConfig config(baud, fc, pt, sb);
  SerialPort port(ctx, dev_null, config);

  EXPECT_FALSE(port.is_open());
  EXPECT_THROW(port.open(), asio::system_error);
  EXPECT_FALSE(port.is_open());

  // Without the port open, should return false
  EXPECT_FALSE(port.send_break());

  ctx.waitForExit();
}

// Below test requires valid serial port
/*
TEST(SerialDriverTest, SendBreakWhenWhileOpen)
{
  IoContext ctx;
  SerialPortConfig config(baud, fc, pt, sb);
  SerialPort port(ctx, dev_name, config);

  EXPECT_FALSE(port.is_open());
  EXPECT_NO_THROW(port.open());
  EXPECT_TRUE(port.is_open());

  // With the port open, expect it to work
  EXPECT_TRUE(port.send_break());

  ctx.waitForExit();
}
*/
