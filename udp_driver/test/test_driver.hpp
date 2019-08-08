// Copyright 2018 Apex.AI, Inc.
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

// Co-developed by Tier IV, Inc. and Apex.AI, Inc.

#ifndef TEST_DRIVER_HPP_
#define TEST_DRIVER_HPP_
#include <string>

#include "std_msgs/msg/int32.hpp"
#include "udp_driver/udp_driver_node.hpp"
#include "udp_driver/visibility_control.hpp"

namespace test_udp_driver
{

class UDP_DRIVER_PUBLIC Packet
{
public:
  explicit Packet(int val);
  Packet();
  int value;
};


using TestDriverT = autoware::drivers::udp_driver::UdpDriverNode<Packet, std_msgs::msg::Int32>;

class UDP_DRIVER_PUBLIC TestDriver : public TestDriverT
{
public:
  TestDriver(
    const std::string & node_name,
    const std::string & topic,
    const std::string & ip,
    const uint16_t port);

  int32_t times_init_called() const;
  int32_t get_last_value() const;
  void reset_reset_flag();

protected:
  void init_output(std_msgs::msg::Int32 & output) override;

  bool convert(const Packet & pkt, std_msgs::msg::Int32 & output) override;

  bool get_output_remainder(std_msgs::msg::Int32 & output) override;

private:
  int32_t m_last_value;
  int32_t m_times_init_output_has_been_called;
};  // class TestDriver

}  // namespace test_udp_driver
#endif  // TEST_DRIVER_HPP_
