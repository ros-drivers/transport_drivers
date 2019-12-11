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

#include "test_driver.hpp"

namespace test_serial_driver
{
    Packet::Packet(int32_t val): value(val) {}
    Packet::Packet(): value(0) {}

TestDriver::TestDriver(
  const std::string & node_name,
  const std::string & topic,
  const std::string & device_name,
  const SerialPortConfig & serial_port_config)
: TestDriverT(node_name, topic, device_name, serial_port_config),
  m_last_value(-1),
  m_times_init_output_has_been_called(0)
{
}

int32_t TestDriver::times_init_called() const {return m_times_init_output_has_been_called;}
int32_t TestDriver::get_last_value() const {return m_last_value;}
void TestDriver::reset_reset_flag()
{
  m_last_value = 0;
}

void TestDriver::init_output(std_msgs::msg::Int32 & output)
{
  (void)output;
  m_last_value = 0;
  ++m_times_init_output_has_been_called;
}

bool TestDriver::convert(const Packet & pkt, std_msgs::msg::Int32 & output)
{
  output.data = pkt.value;
  m_last_value = output.data;
  return true;
}

bool TestDriver::get_output_remainder(std_msgs::msg::Int32 & output)
{
  (void)output;
  return false;
}
}  // namespace test_serial_driver
