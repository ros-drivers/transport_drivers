// Copyright 2021 the Autoware Foundation
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
#include "udp_driver/udp_sender_node.hpp"

#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <io_context/io_context.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <lifecycle_msgs/msg/transition.hpp>

using drivers::common::IoContext;
using drivers::udp_driver::UdpReceiverNode;
using drivers::udp_driver::UdpSenderNode;
using lifecycle_msgs::msg::State;
using lifecycle_msgs::msg::Transition;

/// \brief The purpose of this node is to provide both UDP sending
///        and receiving capability using a shared IoContext.
int main(int argc, char ** argv)
{
  // Force flush of the stdout buffer
  setvbuf(stdout, NULL, _IONBF, BUFSIZ);

  bool receiver_started{false};
  bool sender_started{false};

  rclcpp::init(argc, argv);
  rclcpp::executors::SingleThreadedExecutor exec;
  rclcpp::NodeOptions options;
  IoContext ctx{};

  auto receiver_node = std::make_shared<UdpReceiverNode>(options, ctx);
  auto sender_node = std::make_shared<UdpSenderNode>(options, ctx);

  exec.add_node(receiver_node->get_node_base_interface());
  exec.add_node(sender_node->get_node_base_interface());

  if (receiver_node->configure().id() == State::PRIMARY_STATE_INACTIVE) {
    if (receiver_node->activate().id() == State::PRIMARY_STATE_ACTIVE) {
      receiver_started = true;
    } else {
      throw std::runtime_error{"Failed to activate UDP receiver."};
    }
  } else {
    throw std::runtime_error{"Failed to configure UDP receiver."};
  }

  if (sender_node->configure().id() == State::PRIMARY_STATE_INACTIVE) {
    if (sender_node->activate().id() == State::PRIMARY_STATE_ACTIVE) {
      sender_started = true;
    } else {
      throw std::runtime_error{"Failed to activate UDP sender."};
    }
  } else {
    throw std::runtime_error{"Failed to configure UDP sender."};
  }

  if (receiver_started && sender_started) {
    exec.spin();
  }

  rclcpp::shutdown();

  return 0;
}
