// Copyright 2018 Apex.AI, Inc.
// Co-developed by Tier IV, Inc. and Apex.AI, Inc.
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


/// \copyright Copyright 2018 Apex.AI, Inc.
/// \file
/// \brief This file defines the serial_driver_node class.
#ifndef SERIAL_DRIVER__SERIAL_DRIVER_NODE_HPP_
#define SERIAL_DRIVER__SERIAL_DRIVER_NODE_HPP_

#include <chrono>
#include <memory>
#include <string>
#include "helper_functions/crtp.hpp"
#include "boost/asio.hpp"
#include "boost/array.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace autoware
{
namespace drivers
{
/// \brief A template class and associated utilities which encapsulate basic reading from a serial
/// port
namespace serial_driver
{

enum class flow_control_t
{
  hardware,
  software,
  none
};

enum class parity_t
{
  none,
  odd,
  even
};

enum class stop_bits_t
{
  one,
  onepointfive,
  two
};

/// \brief A node which encapsulates the primary functionality of a serial port receiver
/// \tparam PacketT The type of the packet buffer. Typically a container
/// \tparam OutputT The type a packet gets converted/deserialized into. Should be a ROS 2 message
template<typename Derived, typename PacketT, typename OutputT>
class SerialDriverNode : public rclcpp_lifecycle::LifecycleNode,
  public autoware::common::helper_functions::crtp<Derived>
{
public:
  class SerialPortConfig
  {
public:
    /// \brief Default constructor, configuration class
    /// \param[in] baud_rate Baud rate to read from the serial port.
    /// \param[in] flow_control Whether to use hardware, software or no flow control.
    /// \param[in] parity Parity of the serial transmission.
    /// \param[in] stop_bits Stop bits of the serial transmission.
    SerialPortConfig(
      uint32_t baud_rate,
      flow_control_t flow_control,
      parity_t parity,
      stop_bits_t stop_bits
    )
    : baud_rate_(baud_rate),
      flow_control_(flow_control),
      parity_(parity),
      stop_bits_(stop_bits)
    {}

    uint32_t get_baud_rate() const
    {
      return baud_rate_;
    }
    flow_control_t get_flow_control() const
    {
      return flow_control_;
    }
    parity_t get_parity() const
    {
      return parity_;
    }
    stop_bits_t get_stop_bits() const
    {
      return stop_bits_;
    }

private:
    uint32_t baud_rate_;
    flow_control_t flow_control_;
    parity_t parity_;
    stop_bits_t stop_bits_;
  };

  /// \brief Default constructor, starts driver
  /// \param[in] node_name name of the node for rclcpp internals.
  /// \param[in] topic Name of the topic to publish output on.
  /// \param[in] device_name Name of the serial device.
  /// \param[in] history_depth Size of the publisher's queue.
  /// \param[in] history_depth Size of the publisher's queue.
  /// \param[in] serial_port_config config struct with baud_rate, flow_control, parity and
  /// stop_bits params
  /// \throw runtime error if failed to start threads or configure driver.
  SerialDriverNode(
    const std::string & node_name,
    const std::string & topic,
    const std::string & device_name,
    const SerialPortConfig & serial_port_config,
    size_t history_depth = 10)
  : LifecycleNode(node_name),
    m_pub_ptr(this->create_publisher<OutputT>(topic, rclcpp::QoS(history_depth))),
    m_io_service(),
    m_serial_port(m_io_service)
  {
    init_port(device_name, serial_port_config.get_baud_rate(),
      serial_port_config.get_flow_control(),
      serial_port_config.get_parity(), serial_port_config.get_stop_bits());
  }

  /// \brief Constructor
  /// \param[in] node_name Name of node for rclcpp internals.
  /// \param[in] node_namespace Namespace of this node.
  /// \param[in] history_depth Size of the publisher's queue.
  SerialDriverNode(
    const std::string & node_name,
    const std::string & node_namespace,
    size_t history_depth = 10)
  : LifecycleNode(
      node_name,
      node_namespace),
    m_pub_ptr(
      LifecycleNode::create_publisher<OutputT>(declare_parameter("topic").get<std::string>(),
      rclcpp::QoS(history_depth))),
    m_io_service(),
    m_serial_port(m_io_service)
  {
    const std::string & device_name = declare_parameter("device_name").get<std::string>();
    uint32_t baud_rate = declare_parameter("baud_rate").get<uint32_t>();

    std::string flow_control_parameter = declare_parameter("flow_control").get<std::string>();
    flow_control_t flow_control = flow_control_t::none;

    if ("hardware" == flow_control_parameter) {
      flow_control = flow_control_t::hardware;
    } else if ("software" == flow_control_parameter) {
      flow_control = flow_control_t::software;
    } else if ("none" == flow_control_parameter) {
      flow_control = flow_control_t::none;
    } else {
      throw std::domain_error("Unknown value for flow_control: " + flow_control_parameter);
    }

    std::string parity_parameter = declare_parameter("parity").get<std::string>();
    parity_t parity = parity_t::none;

    if ("none" == parity_parameter) {
      parity = parity_t::none;
    } else if ("odd" == parity_parameter) {
      parity = parity_t::odd;
    } else if ("even" == parity_parameter) {
      parity = parity_t::even;
    } else {
      throw std::domain_error("Unknown value for parity: " + parity_parameter);
    }

    std::string stop_bits_parameter = declare_parameter("stop_bits").get<std::string>();
    stop_bits_t stop_bits = stop_bits_t::one;

    if ("1" == stop_bits_parameter) {
      stop_bits = stop_bits_t::one;
    } else if ("1.5" == stop_bits_parameter) {
      stop_bits = stop_bits_t::onepointfive;
    } else if ("2" == stop_bits_parameter) {
      stop_bits = stop_bits_t::two;
    } else {
      throw std::domain_error("Unknown value for parity: " + stop_bits_parameter);
    }

    init_port(device_name, baud_rate, flow_control, parity, stop_bits);
  }

  // brief Main loop: receives data from serial port, publishes to the given topic
  void run(const uint32_t max_iterations = 0U)
  {
    // initialize the output object
    OutputT output;
    this->impl().init_output(output);
    // activate the publisher
    m_pub_ptr->on_activate();

    uint32_t iter = 0U;
    // workaround for rclcpp logging macros with template class
    rclcpp::Logger node_logger = this->get_logger();

    while (rclcpp::ok()) {
      if ((max_iterations != 0U) && (max_iterations == iter)) {
        break;
      }
      ++iter;
      try {
        PacketT pkt;
        (void) get_packet(pkt, m_serial_port);

        // message received, convert and publish
        if (this->impl().convert(pkt, output)) {
          m_pub_ptr->publish(output);
          while (this->impl().get_output_remainder(output)) {
            m_pub_ptr->publish(output);
          }
        }
      } catch (const std::exception & e) {
        RCLCPP_WARN(node_logger, e.what());
        // And then just continue running
      } catch (...) {
        // Something really weird happened and I can't handle it here
        RCLCPP_WARN(node_logger, "Unknown exception occured in SerialDriverNode");
        throw;
      }
    }
  }

private:
  /// \brief Receives a package via serial port and returns the length of the data in the buffer
  size_t get_packet(PacketT & pkt, boost::asio::serial_port & port)
  {
    boost::system::error_code serial_error;
    constexpr size_t max_data_size = 64 * 1024;
    const size_t len = boost::asio::read(port, boost::asio::buffer(&pkt,
        max_data_size), boost::asio::transfer_exactly(sizeof(pkt)),
        serial_error);

    if (serial_error && serial_error != boost::asio::error::message_size) {
      throw boost::system::system_error(serial_error);
    }

    return len;
  }

  void init_port(
    const std::string & device_name, uint32_t baud_rate, flow_control_t flow_control,
    parity_t parity, stop_bits_t stop_bits)
  {
    m_serial_port.open(device_name);
    m_serial_port.set_option(boost::asio::serial_port_base::baud_rate(baud_rate));
    switch (flow_control) {
      case flow_control_t::hardware:
        m_serial_port.set_option(
          boost::asio::serial_port::flow_control(
            boost::asio::serial_port::flow_control::hardware));
        break;
      case flow_control_t::software:
        m_serial_port.set_option(
          boost::asio::serial_port::flow_control(
            boost::asio::serial_port::flow_control::software));
        break;
      case flow_control_t::none:
        m_serial_port.set_option(
          boost::asio::serial_port::flow_control(
            boost::asio::serial_port::flow_control::none));
        break;
      default:
        throw std::domain_error("Unknown value for flow_control");
    }

    switch (parity) {
      case parity_t::none:
        m_serial_port.set_option(
          boost::asio::serial_port::parity(
            boost::asio::serial_port::parity::none));
        break;
      case parity_t::odd:
        m_serial_port.set_option(
          boost::asio::serial_port::parity(
            boost::asio::serial_port::parity::odd));
        break;
      case parity_t::even:
        m_serial_port.set_option(
          boost::asio::serial_port::parity(
            boost::asio::serial_port::parity::even));
        break;
      default:
        throw std::domain_error("Unknown value for parity");
    }

    switch (stop_bits) {
      case stop_bits_t::one:
        m_serial_port.set_option(
          boost::asio::serial_port::stop_bits(
            boost::asio::serial_port::stop_bits::one));
        break;
      case stop_bits_t::onepointfive:
        m_serial_port.set_option(
          boost::asio::serial_port::stop_bits(
            boost::asio::serial_port::stop_bits::onepointfive));
        break;
      case stop_bits_t::two:
        m_serial_port.set_option(
          boost::asio::serial_port::stop_bits(
            boost::asio::serial_port::stop_bits::two));
        break;
      default:
        throw std::domain_error("Unknown value for stop_bits");
    }
  }

  const std::shared_ptr<typename rclcpp_lifecycle::LifecyclePublisher<OutputT>> m_pub_ptr;
  boost::asio::io_service m_io_service;
  boost::asio::serial_port m_serial_port;
};  // class SerialDriverNode
}  // namespace serial_driver
}  // namespace drivers
}  // namespace autoware

#endif  // SERIAL_DRIVER__SERIAL_DRIVER_NODE_HPP_
