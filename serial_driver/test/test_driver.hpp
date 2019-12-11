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

#ifndef TEST_SERIAL_DRIVER_HPP_
#define TEST_SERIAL_DRIVER_HPP_
#include <std_msgs/msg/int32.hpp>
#include "serial_driver/serial_driver_node.hpp"
#include <serial_driver/visibility_control.hpp>
// double quotes here for static analysis

namespace test_serial_driver
{

class SERIAL_DRIVER_PUBLIC Packet
{
public:
    Packet(int32_t val);
    Packet();
      int32_t value;
};


using autoware::drivers::serial_driver::flow_control_t;
using autoware::drivers::serial_driver::parity_t;
using autoware::drivers::serial_driver::stop_bits_t;

class SERIAL_DRIVER_PUBLIC TestDriver : public autoware::drivers::serial_driver::SerialDriverNode<TestDriver, Packet, std_msgs::msg::Int32>
{
public:
  TestDriver(
    const std::string & node_name,
    const std::string & topic,
    const std::string & device_name,
    const SerialPortConfig & serial_port_config);

  int32_t times_init_called() const;
  int32_t get_last_value() const;
  void reset_reset_flag();

  void init_output(std_msgs::msg::Int32 & output);

  bool convert(const Packet & pkt, std_msgs::msg::Int32 & output);

  bool get_output_remainder(std_msgs::msg::Int32 & output);

private:
  int32_t m_last_value;
  int32_t m_times_init_output_has_been_called;
};  // class TestDriver

using TestDriverT = autoware::drivers::serial_driver::SerialDriverNode<TestDriver, Packet, std_msgs::msg::Int32>;

}  // namespace test_serial_driver
#endif  // TEST_SERIAL_DRIVER_HPP_
