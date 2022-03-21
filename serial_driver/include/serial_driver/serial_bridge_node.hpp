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

#ifndef SERIAL_DRIVER__SERIAL_BRIDGE_NODE_HPP_
#define SERIAL_DRIVER__SERIAL_BRIDGE_NODE_HPP_

#include "serial_driver/serial_driver.hpp"

#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <lifecycle_msgs/msg/state.hpp>

#include "msg_converters/converters.hpp"

namespace lc = rclcpp_lifecycle;
using LNI = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface;
using std_msgs::msg::UInt8MultiArray;

namespace drivers
{
namespace serial_driver
{

/// \brief SerialBridgeNode class which can send and receive serial data
class SerialBridgeNode final
  : public lc::LifecycleNode
{
public:
  /// \brief Default constructor
  /// \param[in] options Options for the node
  explicit SerialBridgeNode(const rclcpp::NodeOptions & options);

  /// \brief Constructor which accepts IoContext
  /// \param[in] options Options for the node
  /// \param[in] ctx A shared IoContext
  SerialBridgeNode(
    const rclcpp::NodeOptions & options,
    const IoContext & ctx);

  /// \brief Destructor - required to manage owned IoContext
  ~SerialBridgeNode();

  /// \brief Callback from transition to "configuring" state.
  /// \param[in] state The current state that the node is in.
  LNI::CallbackReturn on_configure(const lc::State & state) override;

  /// \brief Callback from transition to "activating" state.
  /// \param[in] state The current state that the node is in.
  LNI::CallbackReturn on_activate(const lc::State & state) override;

  /// \brief Callback from transition to "deactivating" state.
  /// \param[in] state The current state that the node is in.
  LNI::CallbackReturn on_deactivate(const lc::State & state) override;

  /// \brief Callback from transition to "unconfigured" state.
  /// \param[in] state The current state that the node is in.
  LNI::CallbackReturn on_cleanup(const lc::State & state) override;

  /// \brief Callback from transition to "shutdown" state.
  /// \param[in] state The current state that the node is in.
  LNI::CallbackReturn on_shutdown(const lc::State & state) override;

  /// \brief Callback for sending a raw serial message
  void subscriber_callback(const UInt8MultiArray::SharedPtr msg);

  /// \breif Callback for when serial data are received
  void receive_callback(const std::vector<uint8_t> & buffer, const size_t & bytes_transferred);

private:
  void get_params();

  std::unique_ptr<IoContext> m_owned_ctx{};
  std::string m_device_name{};
  std::unique_ptr<SerialPortConfig> m_device_config;
  std::unique_ptr<SerialDriver> m_serial_driver;
  lc::LifecyclePublisher<UInt8MultiArray>::SharedPtr m_publisher;
  rclcpp::Subscription<UInt8MultiArray>::SharedPtr m_subscriber;
};  // class SerialBridgeNode

}  // namespace serial_driver
}  // namespace drivers

#endif  // SERIAL_DRIVER__SERIAL_BRIDGE_NODE_HPP_
