// Copyright 2018 Apex.AI, Inc.
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

// Co-developed by Tier IV, Inc. and Apex.AI, Inc.

/// \file
/// \brief This file defines the udp_driver_node class.
#ifndef UDP_DRIVER__UDP_DRIVER_NODE_HPP_
#define UDP_DRIVER__UDP_DRIVER_NODE_HPP_

#include <chrono>
#include <memory>
#include <string>
#include "boost/asio.hpp"
#include "boost/array.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

namespace autoware
{
namespace drivers
{
/// \brief A template class and associated utilities which encapsulate basic reading from UDP
namespace udp_driver
{

/// \brief A node which encapsulates the primary functionality of a UDP receiver
/// \tparam PacketT The type of the packet buffer. Typically a container
/// \tparam OutputT The type a packet gets converted/deserialized into. Should be a ROS 2 message
template<typename PacketT, typename OutputT>
class UdpDriverNode : public rclcpp_lifecycle::LifecycleNode
{
public:
  /// \brief Default constructor, starts driver
  /// \param[in] node_name name of the node for rclcpp internals
  /// \param[in] topic Name of the topic to publish output on
  /// \param[in] ip Expected IP of UDP packets
  /// \param[in] port Port that this driver listens to (i.e. sensor device at ip writes to port)
  /// \throw runtime error if failed to start threads or configure driver
  UdpDriverNode(
    const std::string & node_name,
    const std::string & topic,
    const std::string & ip,
    const uint16_t port)
  : LifecycleNode(node_name),
    m_pub_ptr(this->create_publisher<OutputT>(topic, rclcpp::QoS(10))),
    m_io_service(),
    m_udp_socket(m_io_service,
      boost::asio::ip::udp::endpoint(boost::asio::ip::address::from_string(ip), port)) {}

  /// \brief Constructor
  /// \param[in] node_name Name of node for rclcpp internals
  /// \param[in] node_namespace Namespace of this node
  UdpDriverNode(
    const std::string & node_name,
    const std::string & node_namespace)
  : LifecycleNode(
      node_name,
      node_namespace,
      rclcpp::NodeOptions()),
    // TODO(esteve): Pass empty NodeOptions as workaround for https://github.com/ros2/rclcpp/pull/775
    m_pub_ptr(
      LifecycleNode::create_publisher<OutputT>(declare_parameter("topic").get<std::string>(),
      rclcpp::QoS(10))),
    m_io_service(),
    m_udp_socket(m_io_service, boost::asio::ip::udp::endpoint(
        boost::asio::ip::address::from_string(declare_parameter("ip").get<std::string>()),
        static_cast<uint16_t>(declare_parameter("port").get<uint16_t>()))) {}


  // brief Main loop: receives data from UDP, publishes to the given topic
  void run(const uint32_t max_iterations = 0U)
  {
    // initialize the output object
    OutputT output;
    init_output(output);
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
        (void) get_packet(pkt, m_udp_socket);

        // message received, convert and publish
        if (this->convert(pkt, output)) {
          m_pub_ptr->publish(output);
          while (this->get_output_remainder(output)) {
            m_pub_ptr->publish(output);
          }
        }
      } catch (const std::exception & e) {
        RCLCPP_WARN(node_logger, e.what());
        // And then just continue running
      } catch (...) {
        // Something really weird happened and I can't handle it here
        RCLCPP_WARN(node_logger, "Unknown exception occured in UdpDriverNode");
        throw;
      }
    }
  }

protected:
  /// \brief This method is called on the output just as the node execution thread is started.
  ///        The main use case is preallocating unbounded messages (e.g. PointCloud2), or setting up
  ///        headers.
  /// \param[inout] output The message to get initialized
  virtual void init_output(OutputT & output) = 0;
  /// \brief Converts a packet into an output, updates any stateful information
  /// \param[in] pkt The packet type to be deserialized
  /// \param[out] output Holds the output, if necessary
  /// \return Whether or not an output has been written to (e.g. if sufficient points have been
  ///         deserialized for a full PointCloud)
  virtual bool convert(const PacketT & pkt, OutputT & output) = 0;
  /// \brief Gets the remaining outputs from a packet. This supports the use case i.e. if a packet
  ///        contains information for 500 points, but OutputT can only hold 300 points.
  /// \param[out] output The extra output gets written into this variable
  /// \return True if there are more outputs that must be published
  ///         False if there are no more outputs to publish.
  virtual bool get_output_remainder(OutputT & output) = 0;

private:
  /// \brief Receives a package via UDP and returns the length of the data in the buffer
  size_t get_packet(PacketT & pkt, boost::asio::ip::udp::socket & socket)
  {
    boost::system::error_code udp_error;
    boost::asio::ip::udp::endpoint sender_endpoint;
    constexpr size_t max_data_size = 64 * 1024;
    const size_t len = socket.receive_from(boost::asio::buffer(&pkt, max_data_size),
        sender_endpoint, 0, udp_error);

    if (udp_error && udp_error != boost::asio::error::message_size) {
      throw boost::system::system_error(udp_error);
    }

    return len;
  }

  const std::shared_ptr<typename rclcpp_lifecycle::LifecyclePublisher<OutputT>> m_pub_ptr;
  boost::asio::io_service m_io_service;
  boost::asio::ip::udp::socket m_udp_socket;
};  // class UdpDriverNode
}  // namespace udp_driver
}  // namespace drivers
}  // namespace autoware

#endif  // UDP_DRIVER__UDP_DRIVER_NODE_HPP_
