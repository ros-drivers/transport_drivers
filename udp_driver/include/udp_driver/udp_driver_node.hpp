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

#include "converters.hpp"
#include "udp_driver.hpp"

namespace autoware
{
namespace drivers
{
/// \brief A template class and associated utilities which encapsulate
///        basic reading and writing of UDP socket
namespace udp_driver
{

/// \brief A node which encapsulates the primary functionality of a UDP receiver
/// \tparam PacketT The type of the packet buffer. Typically a container
/// \tparam OutputT The type a packet gets converted/deserialized into. Should be a ROS 2 message
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
    IoContext & ctx)
  : Node(node_name, options), m_udp_driver(new UdpDriver(ctx)) {}

  ~UdpDriverNode()
  {
    std::cout << "[UdpDriverNode::~UdpDriverNode] INFO => Destructing..." << std::endl;
  }

  void initialize_sender(const std::string & ip, int16_t port)
  {
    m_udp_driver->initialize_sender(ip, port);
    if (!m_udp_driver->sender()->isOpen()) {
      m_udp_driver->sender()->open();
    }

    createSubscribers();
  }

  void initialize_receiver(const std::string & ip, uint16_t port)
  {
    createPublishers();

    m_udp_driver->initialize_receiver(ip, port);
    m_udp_driver->receiver()->open();
    m_udp_driver->receiver()->bind();
    m_udp_driver->receiver()->asyncReceive(
      boost::bind(&UdpDriverNode::receiver_callback, this, _1));
  }

private:
  std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Int32>> publisher() const
  {
    return m_publisher;
  }

  void createPublishers()
  {
    m_publisher = this->create_publisher<std_msgs::msg::Int32>("udp_read", rclcpp::QoS(100));
  }

  void createSubscribers()
  {
    m_subscriber = this->create_subscription<std_msgs::msg::Int32>(
      "udp_write",
      rclcpp::QoS(rclcpp::KeepLast(10)).best_effort(),
      std::bind(&UdpDriverNode::subscriber_callback, this, std::placeholders::_1));
  }

  void receiver_callback(const MutSocketBuffer & buffer)
  {
    std::cout << "[UdpDriverNode::receiver_callback] " <<
      *reinterpret_cast<int32_t *>(buffer.data()) << std::endl;

    std_msgs::msg::Int32 out;
    autoware::msgs::convertToRosMessage(buffer, out);

    m_publisher->publish(out);
  }

  void subscriber_callback(std_msgs::msg::Int32::SharedPtr msg)
  {
    std::cout << "[UdpDriverNode::subscriber_callback] " << msg->data << std::endl;

    MutSocketBuffer out;
    autoware::msgs::convertFromRosMessage(msg, out);

    m_udp_driver->sender()->asyncSend(out);
  }

  std::shared_ptr<UdpDriver> m_udp_driver;
  std::shared_ptr<typename rclcpp::Publisher<std_msgs::msg::Int32>> m_publisher;
  std::shared_ptr<typename rclcpp::Subscription<std_msgs::msg::Int32>> m_subscriber;
};  // class UdpDriverNode
}  // namespace udp_driver
}  // namespace drivers
}  // namespace autoware
#endif  // UDP_DRIVER__UDP_DRIVER_NODE_HPP_
