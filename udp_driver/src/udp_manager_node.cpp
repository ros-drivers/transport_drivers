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

#include <udp_manager_node.hpp>
#include <iostream>
#include <memory>

namespace udp_driver
{
  UdpManagerNode::UdpManagerNode(const rclcpp::NodeOptions & options)
  : Node("udp_manager_node", options)
  {
    RCLCPP_INFO(this->get_logger(), "Initializing udp send services");
    RCLCPP_INFO(this->get_logger(), "Initializing udp driver creation services");

  create_udp_driver_ =
    this->create_service<udp_msgs::srv::UdpSocket>(
    "create_udp_driver",
    std::bind(
      &UdpManagerNode::create_driver_handler,
      this, std::placeholders::_1, std::placeholders::_2));
}

void UdpManagerNode::create_driver_handler(
  const std::shared_ptr<udp_msgs::srv::UdpSocket::Request> request,
  std::shared_ptr<udp_msgs::srv::UdpSocket::Response> response)
{
  RCLCPP_INFO(this->get_logger(), "Creating a socket");
}
}  // namespace udp_driver
RCLCPP_COMPONENTS_REGISTER_NODE(udp_driver::UdpManagerNode)
