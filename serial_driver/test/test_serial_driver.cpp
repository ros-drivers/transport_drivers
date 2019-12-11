// Copyright 2018 Apex.AI, Inc.
// Co-developed by Tier IV, Inc. and Apex.AI, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "gtest/gtest.h"
#include "rclcpp/rclcpp.hpp"
#include "test_driver.hpp"

#if defined(__linux__)
#include <pty.h>
#else
#include <util.h>
#endif

using test_serial_driver::TestDriver;
using test_serial_driver::Packet;
using test_serial_driver::flow_control_t;
using test_serial_driver::parity_t;
using test_serial_driver::stop_bits_t;

namespace
{
class serial_driver : public ::testing::Test
{
protected:
  virtual void SetUp() {
    if (openpty(&master_fd, &slave_fd, name, NULL, NULL) == -1) {
      perror("openpty");
      exit(127);
    }

    ASSERT_TRUE(master_fd > 0);
    ASSERT_TRUE(slave_fd > 0);
    ASSERT_TRUE(std::string(name).length() > 0);
  }

  int master_fd;
  int slave_fd;
  char name[100];
};
}  // namespace


//tests serial_driver_node's get_packet function which receives serial packages
TEST_F(serial_driver, basic)
{
  //rclcpp::init required to start the node
  rclcpp::init(0, nullptr);

  //setting values to send
  std::vector<int32_t> values(10);
  std::generate(values.begin(), values.end(), [n = 0] () mutable { return n++; });

  TestDriver driver(
    "serial_driver_node",
    "serial_topic",
    name,
    TestDriver::SerialPortConfig {38400, flow_control_t::software, parity_t::even, stop_bits_t::one}
    );

  for (auto val : values) {
    write(master_fd, reinterpret_cast<char *>(&val), sizeof(val));
    driver.run(1U);
    EXPECT_EQ(driver.get_last_value(), val);
  }
}
