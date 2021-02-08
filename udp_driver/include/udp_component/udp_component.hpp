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

#ifndef UDP_COMPONENT__UDP_COMPONENT_HPP_
#define UDP_COMPONENT__UDP_COMPONENT_HPP_

#include <rclcpp/rclcpp.hpp>

#include <map>
#include <memory>
#include <vector>

#include "udp_msgs/msg/udp_packet.hpp"
#include "udp_driver/udp_driver_node.hpp"
#include "udp_driver/visibility_control.hpp"

namespace udp_component
{

using Packet = std::vector<uint8_t>;

using UdpDriverT = autoware::drivers::udp_driver::UdpDriverNode<Packet, udp_msgs::msg::UdpPacket>;

class UdpComponent : public UdpDriverT
{
public:
  UDP_DRIVER_PUBLIC
  explicit UdpComponent(const rclcpp::NodeOptions & options);

protected:
  void init_output(udp_msgs::msg::UdpPacket & output) override {return;};
  bool convert(const Packet & pkt, udp_msgs::msg::UdpPacket & output) override;
  bool get_output_remainder(udp_msgs::msg::UdpPacket & output) override {return false;};

private:
  std::string address_;
  uint16_t port_;

  std::thread reader_thread;
};  // class UdpComponent
}  //  namespace udp_component
#endif  // UDP_COMPONENT__UDP_COMPONENT_HPP_
