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

#include "udp_driver/udp_socket.hpp"

#include <iostream>
#include <utility>
#include <string>
#include <system_error>
#include <vector>

#include "asio.hpp"
#include "rclcpp/logging.hpp"

namespace drivers
{
namespace udp_driver
{

UdpSocket::UdpSocket(
  const IoContext & ctx,
  const std::string & ip,
  uint16_t port)
: m_ctx(ctx),
  m_udp_socket(ctx.ios()),
  m_endpoint(address::from_string(ip), port)
{
  m_recv_buffer.resize(m_recv_buffer_size);
}

UdpSocket::~UdpSocket()
{
  close();
}

std::size_t UdpSocket::send(const MutBuffer & buff)
{
  try {
    return m_udp_socket.send_to(buff, m_endpoint);
  } catch (const std::system_error & error) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("UdpSocket::send"), error.what());
    return -1;
  }
}

size_t UdpSocket::receive(const MutBuffer & buff)
{
  asio::error_code error;
  asio::ip::udp::endpoint sender_endpoint;

  std::size_t len = m_udp_socket.receive_from(
    buff,
    m_endpoint,
    0,
    error);

  if (error && error != asio::error::message_size) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("UdpSocket::receive"), error.message());
    return -1;
  }
  return len;
}

void UdpSocket::asyncSend(const std::vector<uint8_t> & buff)
{
  m_udp_socket.async_send_to(
    asio::buffer(buff), m_endpoint,
    [this](std::error_code error, std::size_t bytes_transferred)
    {
      asyncSendHandler(error, bytes_transferred);
    });
}

void UdpSocket::asyncReceive(Functor func)
{
  m_func = std::move(func);
  m_udp_socket.async_receive_from(
    asio::buffer(m_recv_buffer),
    m_endpoint,
    [this](std::error_code error, std::size_t bytes_transferred)
    {
      asyncReceiveHandler(error, bytes_transferred);
    });
}

void UdpSocket::asyncSendHandler(
  const asio::error_code & error,
  std::size_t bytes_transferred)
{
  (void)bytes_transferred;
  if (error) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("UdpSocket::asyncSendHandler"), error.message());
  }
}

void UdpSocket::asyncReceiveHandler(
  const asio::error_code & error,
  std::size_t bytes_transferred)
{
  (void)bytes_transferred;
  if (error) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("UdpSocket::asyncReceiveHandler"), error.message());
    return;
  }

  if (bytes_transferred > 0 && m_func) {
    m_recv_buffer.resize(bytes_transferred);
    m_func(m_recv_buffer);
    m_recv_buffer.resize(m_recv_buffer_size);
    m_udp_socket.async_receive_from(
      asio::buffer(m_recv_buffer),
      m_endpoint,
      [this](std::error_code error, std::size_t bytes_tf)
      {
        m_recv_buffer.resize(bytes_tf);
        asyncReceiveHandler(error, bytes_tf);
      });
  }
}

std::string UdpSocket::ip() const
{
  return m_endpoint.address().to_string();
}

uint16_t UdpSocket::port() const
{
  return m_endpoint.port();
}

void UdpSocket::open()
{
  m_udp_socket.open(udp::v4());
}

void UdpSocket::close()
{
  asio::error_code error;
  m_udp_socket.close(error);
  if (error) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("UdpSocket::close"), error.message());
  }
}

bool UdpSocket::isOpen() const
{
  return m_udp_socket.is_open();
}

void UdpSocket::bind()
{
  m_udp_socket.bind(m_endpoint);
}

}  // namespace udp_driver
}  // namespace drivers
