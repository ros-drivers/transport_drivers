#include <udp_manager_node.hpp>
#include <iostream>
#include <memory>

namespace udp_driver
{
  UdpManagerNode::UdpManagerNode()
  : Node("udp_manager_node")
  {
    RCLCPP_INFO(this->get_logger(), "Initializing udp send services");
    RCLCPP_INFO(this->get_logger(), "Initializing udp driver creation services");

    create_udp_driver_ =
              this->create_service<udp_msgs::srv::UdpSocket>("create_udp_driver",
                std::bind(&UdpManagerNode::create_driver_handler,
                this, std::placeholders::_1, std::placeholders::_2));
  }

  void UdpManagerNode::create_driver_handler(
      const std::shared_ptr<udp_msgs::srv::UdpSocket::Request> request,
      std::shared_ptr<udp_msgs::srv::UdpSocket::Response> response)
  {
      RCLCPP_INFO(this->get_logger(), "Creating a socket");
  }
}  // namespace udp_driver
