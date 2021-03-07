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

#ifndef UDP_DRIVER__UDP_SENDER_NODE_HPP_
#define UDP_DRIVER__UDP_SENDER_NODE_HPP_

#include "udp_driver/udp_driver.hpp"

#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <memory>
#include <string>

#include "msg_converters/converters.hpp"

namespace drivers
{
namespace udp_driver
{

class UdpSenderNode : public rclcpp::Node
{
public:
  UdpSenderNode(
    const std::string & node_name,
    const rclcpp::NodeOptions & options,
    IoContext & ctx);

  void init_sender(const std::string & ip, int16_t port);
  void init_receiver(const std::string & ip, uint16_t port);

private:
  void createPublishers();
  void createSubscribers();

  void receiver_callback(const MutSocketBuffer & buffer);
  void subscriber_callback(std_msgs::msg::Int32::SharedPtr msg);

  std::shared_ptr<UdpDriver> m_udp_driver;
  std::shared_ptr<typename rclcpp::Publisher<std_msgs::msg::Int32>> m_publisher;
  std::shared_ptr<typename rclcpp::Subscription<std_msgs::msg::Int32>> m_subscriber;
};  // class UdpSenderNode

}  // namespace udp_driver
}  // namespace drivers

#endif  // UDP_DRIVER__UDP_SENDER_NODE_HPP_
