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

#include "serial_driver/serial_bridge_node.hpp"

#include <memory>
#include <string>
#include <vector>

namespace lc = rclcpp_lifecycle;
using LNI = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface;
using lifecycle_msgs::msg::State;

namespace drivers
{
namespace serial_driver
{

SerialBridgeNode::SerialBridgeNode(const rclcpp::NodeOptions & options)
: lc::LifecycleNode("serial_bridge_node", options),
  m_owned_ctx{new IoContext(2)},
  m_serial_driver{new SerialDriver(*m_owned_ctx)}
{
  get_params();
}

SerialBridgeNode::SerialBridgeNode(
  const rclcpp::NodeOptions & options,
  const IoContext & ctx)
: lc::LifecycleNode("serial_bridge_node", options),
  m_serial_driver{new SerialDriver(ctx)}
{
  get_params();
}

SerialBridgeNode::~SerialBridgeNode()
{
  if (m_owned_ctx) {
    m_owned_ctx->waitForExit();
  }
}

LNI::CallbackReturn SerialBridgeNode::on_configure(const lc::State & state)
{
  (void)state;

  // Create Publisher
  m_publisher = this->create_publisher<UInt8MultiArray>(
    "serial_read", rclcpp::QoS{100});

  try {
    m_serial_driver->init_port(m_device_name, *m_device_config);
    if (!m_serial_driver->port()->is_open()) {
      m_serial_driver->port()->open();
      m_serial_driver->port()->async_receive(
        std::bind(
          &SerialBridgeNode::receive_callback, this, std::placeholders::_1,
          std::placeholders::_2));
    }
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(
      get_logger(), "Error creating serial port: %s - %s",
      m_device_name.c_str(), ex.what());
    return LNI::CallbackReturn::FAILURE;
  }

  // Create Subscriber
  auto qos = rclcpp::QoS(rclcpp::KeepLast(32)).best_effort();
  auto callback = std::bind(&SerialBridgeNode::subscriber_callback, this, std::placeholders::_1);

  m_subscriber = this->create_subscription<UInt8MultiArray>(
    "serial_write", qos, callback);

  RCLCPP_DEBUG(get_logger(), "Serial port successfully configured.");

  return LNI::CallbackReturn::SUCCESS;
}

LNI::CallbackReturn SerialBridgeNode::on_activate(const lc::State & state)
{
  (void)state;
  m_publisher->on_activate();
  RCLCPP_DEBUG(get_logger(), "Serial bridge activated.");
  return LNI::CallbackReturn::SUCCESS;
}

LNI::CallbackReturn SerialBridgeNode::on_deactivate(const lc::State & state)
{
  (void)state;
  m_publisher->on_deactivate();
  RCLCPP_DEBUG(get_logger(), "Serial bridge deactivated.");
  return LNI::CallbackReturn::SUCCESS;
}

LNI::CallbackReturn SerialBridgeNode::on_cleanup(const lc::State & state)
{
  (void)state;
  m_serial_driver->port()->close();
  m_publisher.reset();
  m_subscriber.reset();
  RCLCPP_DEBUG(get_logger(), "Serial bridge cleaned up.");
  return LNI::CallbackReturn::SUCCESS;
}

LNI::CallbackReturn SerialBridgeNode::on_shutdown(const lc::State & state)
{
  (void)state;
  RCLCPP_DEBUG(get_logger(), "Serial bridge shutting down.");
  return LNI::CallbackReturn::SUCCESS;
}

void SerialBridgeNode::get_params()
{
  uint32_t baud_rate{};
  auto fc = FlowControl::NONE;
  auto pt = Parity::NONE;
  auto sb = StopBits::ONE;

  try {
    m_device_name = declare_parameter<std::string>("device_name", "");
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The device name provided was invalid");
    throw ex;
  }

  try {
    baud_rate = declare_parameter<int>("baud_rate", 0);
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The baud_rate provided was invalid");
    throw ex;
  }

  try {
    const auto fc_string = declare_parameter<std::string>("flow_control", "");

    if (fc_string == "none") {
      fc = FlowControl::NONE;
    } else if (fc_string == "hardware") {
      fc = FlowControl::HARDWARE;
    } else if (fc_string == "software") {
      fc = FlowControl::SOFTWARE;
    } else {
      throw std::invalid_argument{
              "The flow_control parameter must be one of: none, software, or hardware."};
    }
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The flow_control provided was invalid");
    throw ex;
  }

  try {
    const auto pt_string = declare_parameter<std::string>("parity", "");

    if (pt_string == "none") {
      pt = Parity::NONE;
    } else if (pt_string == "odd") {
      pt = Parity::ODD;
    } else if (pt_string == "even") {
      pt = Parity::EVEN;
    } else {
      throw std::invalid_argument{
              "The parity parameter must be one of: none, odd, or even."};
    }
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The parity provided was invalid");
    throw ex;
  }

  try {
    const auto sb_string = declare_parameter<std::string>("stop_bits", "");

    if (sb_string == "1" || sb_string == "1.0") {
      sb = StopBits::ONE;
    } else if (sb_string == "1.5") {
      sb = StopBits::ONE_POINT_FIVE;
    } else if (sb_string == "2" || sb_string == "2.0") {
      sb = StopBits::TWO;
    } else {
      throw std::invalid_argument{
              "The stop_bits parameter must be one of: 1, 1.5, or 2."};
    }
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The stop_bits provided was invalid");
    throw ex;
  }

  m_device_config = std::make_unique<SerialPortConfig>(baud_rate, fc, pt, sb);
}

void SerialBridgeNode::receive_callback(
  const std::vector<uint8_t> & buffer,
  const size_t & bytes_transferred)
{
  UInt8MultiArray out;
  drivers::common::to_msg(buffer, out, bytes_transferred);
  m_publisher->publish(out);
}

void SerialBridgeNode::subscriber_callback(const UInt8MultiArray::SharedPtr msg)
{
  if (this->get_current_state().id() == State::PRIMARY_STATE_ACTIVE) {
    std::vector<uint8_t> out;
    drivers::common::from_msg(msg, out);
    m_serial_driver->port()->async_send(out);
  }
}

}  // namespace serial_driver
}  // namespace drivers

#include <rclcpp_components/register_node_macro.hpp>  // NOLINT
RCLCPP_COMPONENTS_REGISTER_NODE(drivers::serial_driver::SerialBridgeNode)
