// Copyright 2018 Apex.AI, Inc.
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
//
// Co-developed by Tier IV, Inc. and Apex.AI, Inc.

#include "serial_component/serial_component.hpp"

#include <string>

namespace serial_component
{

Packet::Packet(int32_t val)
: value(val) {}
Packet::Packet()
: value(0) {}

SerialComponent::SerialComponent(
  const rclcpp::NodeOptions & options)
: SerialDriverT("serial_component", options)
{
  RCLCPP_INFO(this->get_logger(), "Initializing serial_component");
}

int32_t SerialComponent::times_init_called() const {return m_times_init_output_has_been_called;}
int32_t SerialComponent::get_last_value() const {return m_last_value;}
void SerialComponent::reset_reset_flag()
{
  m_last_value = 0;
}

void SerialComponent::init_output(std_msgs::msg::Int32 & output)
{
  (void)output;
  m_last_value = 0;
  ++m_times_init_output_has_been_called;
}

bool SerialComponent::convert(const Packet & pkt, std_msgs::msg::Int32 & output)
{
  output.data = pkt.value;
  m_last_value = output.data;
  return true;
}

bool SerialComponent::get_output_remainder(std_msgs::msg::Int32 & output)
{
  (void)output;
  return false;
}
}  // namespace serial_component

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(serial_component::SerialComponent)
