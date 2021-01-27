#ifndef UDP_DRIVER__UDP_MANAGER_NODE_HPP_
#define UDP_DRIVER__UDP_MANAGER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>

#include <map>
#include <memory>

#include "udp_msgs/msg/udp_packet.hpp"
#include "udp_msgs/srv/udp_send.hpp"
#include "udp_msgs/srv/udp_socket.hpp"

namespace udp_driver
{

class UdpManagerNode : public rclcpp::Node
{
  public:
    UdpManagerNode();
    ~UdpManagerNode(){}

  private:
    rclcpp::Service<udp_msgs::srv::UdpSocket>::SharedPtr create_udp_driver_;
    std::map<uint16_t, std::shared_ptr<udp_msgs::srv::UdpSocket>> active_udp_drivers_;

    void create_driver_handler(
      const std::shared_ptr<udp_msgs::srv::UdpSocket::Request> request,
      const std::shared_ptr<udp_msgs::srv::UdpSocket::Response> response);
};
}  //  namespace udp_driver

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<udp_driver::UdpManagerNode> node = std::make_shared<udp_driver::UdpManagerNode>();

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "UDP Manager node is up and running");
  rclcpp::spin(node);
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "UDP Manager node is shutting down...");
  rclcpp::shutdown();
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Exiting...");
  return 0;
}
#endif  // UDP_DRIVER__UDP_MANAGER_NODE_HPP_
