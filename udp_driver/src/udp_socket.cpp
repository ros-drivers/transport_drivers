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
}

UdpSocket::~UdpSocket()
{
  std::cout << "[UdpSocket::~UdpSocket] INFO => Destructing..." << std::endl;
  close();
}

std::size_t UdpSocket::send(const MutSocketBuffer & buff)
{
  try {
    return m_udp_socket.send_to(buff, m_endpoint);
  } catch (const boost::system::system_error & error) {
    std::cout << "[UdpSocket::send] Error => " <<
      error.what() << std::endl;
    return -1;
  }
}

size_t UdpSocket::receive(const MutSocketBuffer & buff)
{
  boost::system::error_code error;
  boost::asio::ip::udp::endpoint sender_endpoint;

  std::size_t len = m_udp_socket.receive_from(
    buff,
    m_endpoint,
    0,
    error);

  if (error && error != boost::asio::error::message_size) {
    std::cout << "[UdpSocket::receive] Error => " <<
      error.message() << std::endl;
    return -1;
  }
  return len;
}

void UdpSocket::asyncSend(const MutSocketBuffer & buff)
{
  m_udp_socket.async_send_to(
    buff, m_endpoint, boost::bind(
      &UdpSocket::asyncSendHandler,
      this,
      boost::asio::placeholders::error,
      boost::asio::placeholders::bytes_transferred));
}

void UdpSocket::asyncReceive(Functor func)
{
  m_func = std::move(func);
  m_udp_socket.async_receive_from(
    boost::asio::buffer(m_recv_buffer, m_recv_buffer_size),
    m_endpoint,
    boost::bind(
      &UdpSocket::asyncReceiveHandler, this,
      boost::asio::placeholders::error,
      boost::asio::placeholders::bytes_transferred));
}

void UdpSocket::asyncSendHandler(
  const boost::system::error_code & error,
  std::size_t bytes_transferred)
{
  (void)bytes_transferred;
  if (error) {
    std::cout << "[UdpSocket::asyncSendHandler] Error => " <<
      error.message() << std::endl;
  }
}

void UdpSocket::asyncReceiveHandler(
  const boost::system::error_code & error,
  std::size_t bytes_transferred)
{
  (void)bytes_transferred;
  if (error) {
    std::cout << "[UdpSocket::asyncReceiveHandler] Error => " <<
      error.message() << std::endl;
    return;
  }

  if (bytes_transferred > 0 && m_func) {
    m_func(MutSocketBuffer(m_recv_buffer.data(), bytes_transferred));
    m_udp_socket.async_receive_from(
      boost::asio::buffer(m_recv_buffer, m_recv_buffer_size),
      m_endpoint,
      boost::bind(
        &UdpSocket::asyncReceiveHandler, this,
        boost::asio::placeholders::error,
        boost::asio::placeholders::bytes_transferred));
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
  boost::system::error_code error;
  m_udp_socket.close(error);
  if (error) {
    std::cout << "[UdpSocket::close] Error => " <<
      error.message() << std::endl;
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
