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

#include "udp_driver_node.hpp"

namespace autoware {
namespace drivers {

UdpDriverNode::UdpDriverNode(const std::string &node_name, const rclcpp::NodeOptions &options, IoContext &ctx) :
  Node(node_name, options),
  m_udp_driver(new UdpDriver(ctx)) {
}

UdpDriverNode::~UdpDriverNode() {
  std::cout << "[UdpDriverNode::~UdpDriverNode] INFO => Destructing..." << std::endl;
}

void UdpDriverNode::init_sender(const std::string &ip, int16_t port) {
  m_udp_driver->init_sender(ip, port);
  if (!m_udp_driver->sender()->isOpen()) {
    m_udp_driver->sender()->open();
  }

  createSubscribers();
}

void UdpDriverNode::init_receiver(const std::string &ip, uint16_t port) {
  createPublishers();

  m_udp_driver->init_receiver(ip, port);
  m_udp_driver->receiver()->open();
  m_udp_driver->receiver()->bind();
  m_udp_driver->receiver()->asyncReceive(boost::bind(&UdpDriverNode::receiver_callback, this, _1));
}

void UdpDriverNode::createPublishers() {
  m_publisher = this->create_publisher<std_msgs::msg::Int32>("udp_read",
                                                             rclcpp::QoS(100));
}

void UdpDriverNode::createSubscribers() {
  m_subscriber = this->create_subscription<std_msgs::msg::Int32>("udp_write",
                                                                 rclcpp::QoS(rclcpp::KeepLast(10)).best_effort(),
                                                                 std::bind(&UdpDriverNode::subscriber_callback,
                                                                           this,
                                                                           std::placeholders::_1));
}

void UdpDriverNode::receiver_callback(const MutSocketBuffer &buffer) {
  std::cout << "[UdpDriverNode::receiver_callback] " << *(int32_t *) buffer.data() << std::endl;

  std_msgs::msg::Int32 out;
  autoware::msgs::convertToRos2Message(buffer, out);

  m_publisher->publish(out);
}

void UdpDriverNode::subscriber_callback(std_msgs::msg::Int32::SharedPtr msg) {
  std::cout << "[UdpDriverNode::subscriber_callback] " << msg->data << std::endl;

  MutSocketBuffer out;
  autoware::msgs::convertFromRos2Message(msg, out);

  m_udp_driver->sender()->asyncSend(out);
}

}  // namespace drivers
}  // namespace autoware