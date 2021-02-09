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
// Maintained by LeoDrive, 2021

/// \file
/// \brief This file defines the udp_driver_node class.
#ifndef UDP_DRIVER__UDP_DRIVER_NODE_HPP_
#define UDP_DRIVER__UDP_DRIVER_NODE_HPP_

#include <chrono>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"

#include "udp_sender.h"
#include "udp_receiver.h"

namespace autoware
{
namespace drivers
{
/// \brief A template class and associated utilities which encapsulate basic reading and writing of UDP socket
namespace udp_driver
{

struct Endpoint {
  Endpoint(const std::string &ip, uint16_t port) : m_ip(ip), m_port(port) {}
  const std::string &ip() const { return m_ip; }
  uint16_t port() const { return m_port; }

private:
  std::string m_ip;
  uint16_t m_port;
};

class UdpConfig {
public:
    UdpConfig(const Endpoint &client_endpoint, const Endpoint &binding_endpoint):
            m_client_endpoint(client_endpoint),
            m_binding_endpoint(binding_endpoint) {}

    const Endpoint & client_endpoint() const { return m_client_endpoint; }
    const Endpoint & binding_endpoint() const { return m_binding_endpoint; }

private:
    const Endpoint m_client_endpoint;
    const Endpoint m_binding_endpoint;
};

/// \brief A node which encapsulates the primary functionality of a UDP receiver
/// \tparam PacketT The type of the packet buffer. Typically a container
/// \tparam OutputT The type a packet gets converted/deserialized into. Should be a ROS 2 message
template<typename PacketT, typename OutputT>
class UdpDriverNode : public rclcpp::Node
{
public:
  /// \brief Constructor - Gets config through arguments
  /// \param[in] node_name name of the node for rclcpp internals
  /// \param[in] options rclcpp::NodeOptions instance containing options for the node
  /// \param[in] udp_config An UdpConfig object with the expected IP of UDP packets and the port
  ///            that this driver listens to (i.e. sensor device at ip writes to port)
  /// \throw runtime error if failed to start threads or configure driver
  UdpDriverNode(
    const std::string & node_name,
    const rclcpp::NodeOptions & options,
    const UdpConfig & udp_config)
  : Node(node_name, options),
    m_pub_ptr(this->create_publisher<OutputT>("udp_read", rclcpp::QoS(10))),
    m_sub_ptr(this->create_subscription<OutputT>("/udp_write",
              rclcpp::QoS(rclcpp::KeepLast(10)).best_effort(),
              std::bind(&UdpDriverNode::sender_callback, this, std::placeholders::_1))),
    m_sender(new UdpSender(udp_config.client_endpoint().ip(), udp_config.client_endpoint().port())),
    m_receiver(new UdpReceiver(udp_config.binding_endpoint().ip(), udp_config.binding_endpoint().port())) {}

  /// \brief Constructor - Gets config from ROS parameters
  /// \param[in] node_name Name of node for rclcpp internals
  /// \param[in] options rclcpp::NodeOptions instance containing options for the node
  UdpDriverNode(
    const std::string & node_name,
    const rclcpp::NodeOptions & options)
  : Node(node_name, options),
    m_pub_ptr(Node::create_publisher<OutputT>("udp_read", rclcpp::QoS(10))),
    m_sender(new UdpSender(declare_parameter("ip").get<std::string>(), static_cast<uint16_t>(declare_parameter("port").get<uint16_t>()))),
    m_receiver(new UdpReceiver(declare_parameter("ip").get<std::string>(), static_cast<uint16_t>(declare_parameter("port").get<uint16_t>()))) {
  }

  // brief Main loop: receives data from UDP, publishes to the given topic
  void run(const uint32_t max_iterations = 0U)
  {
    // initialize the output object
    OutputT output;
    init_output(output);

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
        (void) m_receiver->receive_packet(pkt);

        // message received, convert and publish
        if (this->convertTo(pkt, output)) {
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
  virtual bool convertTo(const PacketT & pkt, OutputT & output) = 0;

  /// \brief Converts a ROS2 message into a packet buffer
  /// \param[in] output Holds the output, if necessary
  /// \param[out] pkt The packet type to be deserialized
  /// \return Whether or not an output has been written to a packet buffer
  virtual bool convertFrom(const OutputT & output, PacketT & pkt) = 0;

  /// \brief Gets the remaining outputs from a packet. This supports the use case i.e. if a packet
  ///        contains information for 500 points, but OutputT can only hold 300 points.
  /// \param[out] output The extra output gets written into this variable
  /// \return True if there are more outputs that must be published
  ///         False if there are no more outputs to publish.
  virtual bool get_output_remainder(OutputT & output) = 0;

private:
  void sender_callback(const typename OutputT::SharedPtr msg) {
      PacketT pkt;
      if (this->convertFrom(*msg, pkt)) {
          if (m_sender->send(pkt) == -1) {
              RCLCPP_WARN(this->get_logger(), "Cannot send Packet data on socket.");
          }
      }
  }

  const std::shared_ptr<typename rclcpp::Publisher<OutputT>> m_pub_ptr;
  const std::shared_ptr<typename rclcpp::Subscription<OutputT>> m_sub_ptr;

  std::unique_ptr<UdpSender> m_sender;
  std::unique_ptr<UdpReceiver> m_receiver;
};  // class UdpDriverNode

}  // namespace udp_driver
}  // namespace drivers
}  // namespace autoware

#endif  // UDP_DRIVER__UDP_DRIVER_NODE_HPP_
