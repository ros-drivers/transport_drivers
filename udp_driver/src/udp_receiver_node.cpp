// Copyright 2021 LeoDrive, Copyright 2021 the Autoware Foundation
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

#include "udp_driver/udp_receiver_node.hpp"

#include <memory>
#include <string>
#include <vector>

namespace lc = rclcpp_lifecycle;
using LNI = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface;
using lifecycle_msgs::msg::State;

namespace drivers
{
namespace udp_driver
{

UdpReceiverNode::UdpReceiverNode(const rclcpp::NodeOptions & options)
: lc::LifecycleNode("udp_receiver_node", options),
  m_owned_ctx{new IoContext(1)},
  m_udp_driver{new UdpDriver(*m_owned_ctx)}
{
  get_params();
}

UdpReceiverNode::UdpReceiverNode(
  const rclcpp::NodeOptions & options,
  const IoContext & ctx)
: lc::LifecycleNode("udp_receiver_node", options),
  m_udp_driver{new UdpDriver(ctx)}
{
  get_params();
}

UdpReceiverNode::~UdpReceiverNode()
{
  if (m_owned_ctx) {
    m_owned_ctx->waitForExit();
  }
}

LNI::CallbackReturn UdpReceiverNode::on_configure(const lc::State & state)
{
  (void)state;

  m_publisher = this->create_publisher<udp_msgs::msg::UdpPacket>(
    "udp_read", rclcpp::QoS(100));

  try {
    m_udp_driver->init_receiver(m_ip, m_port);
    m_udp_driver->receiver()->open();
    m_udp_driver->receiver()->bind();
    m_udp_driver->receiver()->asyncReceive(
      std::bind(&UdpReceiverNode::receiver_callback, this, std::placeholders::_1));
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(
      get_logger(), "Error creating UDP receiver: %s:%i - %s",
      m_ip.c_str(), m_port, ex.what());
    return LNI::CallbackReturn::FAILURE;
  }

  RCLCPP_DEBUG(get_logger(), "UDP receiver successfully configured.");

  return LNI::CallbackReturn::SUCCESS;
}

LNI::CallbackReturn UdpReceiverNode::on_activate(const lc::State & state)
{
  (void)state;
  m_publisher->on_activate();
  RCLCPP_DEBUG(get_logger(), "UDP receiver activated.");
  return LNI::CallbackReturn::SUCCESS;
}

LNI::CallbackReturn UdpReceiverNode::on_deactivate(const lc::State & state)
{
  (void)state;
  m_publisher->on_deactivate();
  RCLCPP_DEBUG(get_logger(), "UDP receiver deactivated.");
  return LNI::CallbackReturn::SUCCESS;
}

LNI::CallbackReturn UdpReceiverNode::on_cleanup(const lc::State & state)
{
  (void)state;
  m_udp_driver->receiver()->close();
  m_publisher.reset();
  RCLCPP_DEBUG(get_logger(), "UDP receiver cleaned up.");
  return LNI::CallbackReturn::SUCCESS;
}

LNI::CallbackReturn UdpReceiverNode::on_shutdown(const lc::State & state)
{
  (void)state;
  RCLCPP_DEBUG(get_logger(), "UDP receiver shutting down.");
  return LNI::CallbackReturn::SUCCESS;
}

void UdpReceiverNode::get_params()
{
  try {
    m_ip = declare_parameter<std::string>("ip", "");
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The ip paramter provided was invalid");
    throw ex;
  }

  try {
    m_port = declare_parameter<int>("port", 0);
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The port paramter provided was invalid");
    throw ex;
  }

  RCLCPP_INFO(get_logger(), "ip: %s", m_ip.c_str());
  RCLCPP_INFO(get_logger(), "port: %i", m_port);
}

void UdpReceiverNode::receiver_callback(const std::vector<uint8_t> & buffer)
{
  udp_msgs::msg::UdpPacket out;

  out.header.frame_id = m_ip;
  out.header.stamp = this->now();
  out.address = m_ip;
  out.src_port = m_port;

  out.data = buffer;

  m_publisher->publish(out);
}

}  // namespace udp_driver
}  // namespace drivers

#include <rclcpp_components/register_node_macro.hpp>  // NOLINT
RCLCPP_COMPONENTS_REGISTER_NODE(drivers::udp_driver::UdpReceiverNode)
