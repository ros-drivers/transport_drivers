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
#include <string>
#include <vector>

#include "udp_msgs/msg/udp_packet.hpp"
#include "udp_driver/udp_driver.hpp"

namespace udp_component
{

using Packet = std::vector<uint8_t>;

using autoware::drivers::IoContext;
using autoware::drivers::udp_driver::UdpDriver;

class UdpComponent : public rclcpp::Node
{
public:
  explicit UdpComponent(const rclcpp::NodeOptions & options);
  ~UdpComponent() {}

  const std::string & get_ip() const
  {
    return ip_;
  }

  const uint16_t get_port() const
  {
    return port_;
  }

  const std::string & get_frame_id() const
  {
    return frame_id_;
  }

  void handlePacket(const boost::asio::mutable_buffer & packet);

private:
  std::string ip_;
  uint16_t port_;
  std::string frame_id_;

  IoContext context_;
  UdpDriver udp_driver_;  // TODO(flynneva): make this a vector of drivers?

  rclcpp::Publisher<udp_msgs::msg::UdpPacket>::SharedPtr udp_publisher_;
};  // class UdpComponent
}  //  namespace udp_component
#endif  // UDP_COMPONENT__UDP_COMPONENT_HPP_
