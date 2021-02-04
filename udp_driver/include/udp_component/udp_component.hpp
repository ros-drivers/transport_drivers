// Copyright 2021 Apex.AI, Inc.
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

#ifndef UDP_COMPONENT_HPP_
#define UDP_COMPONENT_HPP_

#include <rclcpp/rclcpp.hpp>

#include <map>
#include <memory>

#include "std_msgs/msg/int32.hpp"
#include "udp_driver/udp_driver_node.hpp"
#include "udp_driver/visibility_control.hpp"

namespace udp_component
{

class Packet
{
public:
  UDP_DRIVER_PUBLIC
  explicit Packet(int val);
  Packet();
  int value;
};

using UdpDriverT = autoware::drivers::udp_driver::UdpDriverNode<Packet, std_msgs::msg::Int32>;

class UdpComponent : public UdpDriverT
{
public:
  UDP_DRIVER_PUBLIC
  explicit UdpComponent(const rclcpp::NodeOptions & options);

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
};  // class UdpComponent
}  //  namespace udp_component
#endif  // UDP_COMPONENT_HPP_
