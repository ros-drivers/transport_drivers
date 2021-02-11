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
#include "udp_component/udp_component.hpp"

#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"


namespace udp_component
{

UdpComponent::UdpComponent(const rclcpp::NodeOptions & options)
: UdpDriverT("udp_component", options)
{
  RCLCPP_INFO(this->get_logger(), "Initializing udp_component");

  // set output using already declared ROS parameters
  this->get_parameter("ip", address_);
  this->get_parameter("port", port_);
  RCLCPP_INFO(this->get_logger(), "ip: %s", address_.c_str());
  RCLCPP_INFO(this->get_logger(), "port: %i", port_);

  RCLCPP_INFO(this->get_logger(), "Starting udp reader thread");
  // start receiving packets on separate thread
  reader_thread = std::thread{[this]() {this->run();}};
}

bool UdpComponent::convert(const udp_component::Packet & pkt, udp_msgs::msg::UdpPacket & output)
{
  RCLCPP_INFO(this->get_logger(), "converter");
  RCLCPP_INFO(this->get_logger(), "packet size: %i", pkt.size());
  RCLCPP_INFO(this->get_logger(), "node name: %s", this->get_name());
  RCLCPP_INFO(this->get_logger(), "output: %s", output.address.c_str());

  // clear out old data
  output.data.clear();
  // resize output vector to incoming packets size
  output.data.resize(pkt.size());
  // set output header
  RCLCPP_INFO(this->get_logger(), "output size: %s", output.data.size());

  // output.header.stamp = this->now();

  // RCLCPP_INFO(this->get_logger(), "packet size: %i", pkt.size());
  // RCLCPP_INFO(this->get_logger(), "output.frame_id: %s", output.header.frame_id.c_str());
  RCLCPP_INFO(this->get_logger(), "output.address: %s", address_.c_str());
  RCLCPP_INFO(this->get_logger(), "output.port: %i", port_);
  RCLCPP_INFO(this->get_logger(), "END CONVERT");
  return true;
}

}  // namespace udp_component

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(udp_component::UdpComponent)
