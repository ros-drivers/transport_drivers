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

#include <string>
#include <thread>
#include <vector>

#include "rclcpp/rclcpp.hpp"


namespace udp_component
{

UdpComponent::UdpComponent(const rclcpp::NodeOptions & options)
: Node("udp_component", options),
  udp_driver_(context_),
  ip_(declare_parameter("ip").get<std::string>()),
  port_(declare_parameter("port").get<uint16_t>()),
  frame_id_(declare_parameter("frame_id").get<std::string>())
{
  udp_publisher_ = this->create_publisher<udp_msgs::msg::UdpPacket>(
                         "p" + std::to_string(this->get_port()), 10);

  RCLCPP_INFO(this->get_logger(), "Initializing udp_component with:");
  RCLCPP_INFO(this->get_logger(), "  ip: %s", this->get_ip().c_str());
  RCLCPP_INFO(this->get_logger(), "  port: %i", this->get_port());
  RCLCPP_INFO(this->get_logger(), "  frame_id: %s", this->get_frame_id().c_str());
  this->udp_driver_.initialize_sender(this->get_ip(), this->get_port());
  this->udp_driver_.initialize_receiver(this->get_ip(), this->get_port());
  RCLCPP_INFO(this->get_logger(), "Opening reciever socket");
  this->udp_driver_.receiver()->open();
  this->udp_driver_.receiver()->bind();
  RCLCPP_INFO(this->get_logger(), "Receive socket is open, listening for packets");
  this->udp_driver_.receiver()->asyncReceive(boost::bind(&UdpComponent::handlePacket, this, _1));
}

void UdpComponent::handlePacket(const boost::asio::mutable_buffer &packet)
{
  udp_msgs::msg::UdpPacket udp_packet;
  // Populate UDP packet ROS message
  udp_packet.header.stamp = this->now();
  udp_packet.header.frame_id = this->get_frame_id();
  udp_packet.address = this->get_ip();
  udp_packet.src_port = this->get_port();

  // copy buffer into ROS msg
  udp_packet.data.resize(packet.size());
  // udp_packet.data = packet.data();
  // udp_packet.data = *reinterpret_cast<std::vector<uint8_t>*>(packet.data());
  // memcpy(&udp_packet.data, (uint8_t*)packet.data(), packet.size());
  // udp_packet.data.assign(packet.data(), packet.data() + packet.size());

  // publish msg
  udp_publisher_->publish(udp_packet);
}
}  // namespace udp_component

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(udp_component::UdpComponent)
