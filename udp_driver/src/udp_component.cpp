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

#include "rclcpp/rclcpp.hpp"

namespace udp_component
{

UdpComponent::UdpComponent(const rclcpp::NodeOptions & options)
: UdpDriverT("udp_component", options)
{
  RCLCPP_INFO(this->get_logger(), "Initializing udp_component");
}

int32_t UdpComponent::times_init_called() const
{
  RCLCPP_INFO(this->get_logger(), "times_init_called");
  return m_times_init_output_has_been_called;
}

int32_t UdpComponent::get_last_value() const
{
  RCLCPP_INFO(this->get_logger(), "get_last_value");
  return m_last_value;
}

void UdpComponent::reset_reset_flag()
{
  RCLCPP_INFO(this->get_logger(), "reset_reset_flag");
  m_last_value = 0;
}

void UdpComponent::init_output(std_msgs::msg::Int32 & output)
{
  RCLCPP_INFO(this->get_logger(), "init output");
  (void)output;
  m_last_value = 0;
  ++m_times_init_output_has_been_called;
}

bool UdpComponent::convert(const Packet & pkt, std_msgs::msg::Int32 & output)
{
  RCLCPP_INFO(this->get_logger(), "converter");
  output.data = pkt.value;
  m_last_value = output.data;
  return true;
}

bool UdpComponent::get_output_remainder(std_msgs::msg::Int32 & output)
{
  RCLCPP_INFO(this->get_logger(), "get output remainder");
  (void)output;
  return false;
}
}  // namespace udp_component

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(udp_component::UdpComponent)
