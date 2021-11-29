// Copyright 2021 LeoDrive.
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

// Developed by LeoDrive, 2021

#include "udp_driver/udp_sender_node.hpp"

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

UdpSenderNode::UdpSenderNode(const rclcpp::NodeOptions & options)
: lc::LifecycleNode("udp_sender_node", options),
  m_owned_ctx{new IoContext(1)},
  m_udp_driver{new UdpDriver(*m_owned_ctx)}
{
  get_params();
}

UdpSenderNode::UdpSenderNode(
  const rclcpp::NodeOptions & options,
  const IoContext & ctx)
: lc::LifecycleNode("udp_sender_node", options),
  m_udp_driver{new UdpDriver(ctx)}
{
  get_params();
}

UdpSenderNode::~UdpSenderNode()
{
  if (m_owned_ctx) {
    m_owned_ctx->waitForExit();
  }
}

LNI::CallbackReturn UdpSenderNode::on_configure(const lc::State & state)
{
  (void)state;

  try {
    m_udp_driver->init_sender(m_ip, m_port);
    if (!m_udp_driver->sender()->isOpen()) {
      m_udp_driver->sender()->open();
    }
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(
      get_logger(), "Error creating UDP sender: %s:%i - %s",
      m_ip.c_str(), m_port, ex.what());
    return LNI::CallbackReturn::FAILURE;
  }

  auto qos = rclcpp::QoS(rclcpp::KeepLast(32)).best_effort();
  auto callback = std::bind(&UdpSenderNode::subscriber_callback, this, std::placeholders::_1);

  m_subscriber = this->create_subscription<udp_msgs::msg::UdpPacket>(
    "udp_write", qos, callback);

  RCLCPP_DEBUG(get_logger(), "UDP sender successfully configured.");

  return LNI::CallbackReturn::SUCCESS;
}

LNI::CallbackReturn UdpSenderNode::on_activate(const lc::State & state)
{
  (void)state;
  RCLCPP_DEBUG(get_logger(), "UDP sender activated.");
  return LNI::CallbackReturn::SUCCESS;
}

LNI::CallbackReturn UdpSenderNode::on_deactivate(const lc::State & state)
{
  (void)state;
  RCLCPP_DEBUG(get_logger(), "UDP sender deactivated.");
  return LNI::CallbackReturn::SUCCESS;
}

LNI::CallbackReturn UdpSenderNode::on_cleanup(const lc::State & state)
{
  (void)state;
  m_udp_driver->sender()->close();
  m_subscriber.reset();
  RCLCPP_DEBUG(get_logger(), "UDP sender cleaned up.");
  return LNI::CallbackReturn::SUCCESS;
}

LNI::CallbackReturn UdpSenderNode::on_shutdown(const lc::State & state)
{
  (void)state;
  RCLCPP_DEBUG(get_logger(), "UDP sender shutting down.");
  return LNI::CallbackReturn::SUCCESS;
}

void UdpSenderNode::get_params()
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

void UdpSenderNode::subscriber_callback(udp_msgs::msg::UdpPacket::SharedPtr msg)
{
  if (this->get_current_state().id() == State::PRIMARY_STATE_ACTIVE) {
    std::vector<uint8_t> out;
    drivers::common::from_msg(msg, out);
    m_udp_driver->sender()->asyncSend(out);
  }
}

}  // namespace udp_driver
}  // namespace drivers

#include <rclcpp_components/register_node_macro.hpp>  // NOLINT
RCLCPP_COMPONENTS_REGISTER_NODE(drivers::udp_driver::UdpSenderNode)
