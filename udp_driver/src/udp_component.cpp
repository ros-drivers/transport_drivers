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
: Node("udp_component", options),
  udp_driver_(context_),
  ip_(declare_parameter("ip").get<std::string>()),
  port_(declare_parameter("port").get<uint16_t>())
{
  RCLCPP_INFO(this->get_logger(), "Initializing udp_component");
  RCLCPP_INFO(this->get_logger(), "ip: %s", this->get_ip().c_str());
  RCLCPP_INFO(this->get_logger(), "port: %i", this->get_port());

  this->udp_driver_.initialize_sender(this->get_ip(), this->get_port());
  this->udp_driver_.initialize_receiver(this->get_ip(), this->get_port());

  RCLCPP_INFO(this->get_logger(), "Initialized sender and receiver socket");
  RCLCPP_INFO(this->get_logger(), "Opening reciever socket");

  this->udp_driver_.receiver()->open();
  this->udp_driver_.receiver()->bind();
  this->udp_driver_.receiver()->asyncReceive(boost::bind(&UdpComponent::handlePacket, this, _1));
}

void UdpComponent::handlePacket(const boost::asio::mutable_buffer &packet)
{
  RCLCPP_INFO(this->get_logger(), "RECEIVED PACKET");
}

}  // namespace udp_component

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(udp_component::UdpComponent)
