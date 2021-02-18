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

#ifndef UDP_DRIVER_UDP_SOCKET_HPP
#define UDP_DRIVER_UDP_SOCKET_HPP

#include <string>
#include <boost/array.hpp>
#include "io_context.hpp"
#include "converters.hpp"

using boost::asio::ip::udp;
using boost::asio::ip::address;

namespace autoware {
namespace drivers {

typedef boost::function<void(const MutSocketBuffer &)> Functor;

class UdpSocket : private boost::noncopyable {
public:
  UdpSocket(const IoContext &ctx, const std::string &ip, uint16_t port);
  ~UdpSocket();

  std::string ip() const;
  uint16_t port() const;

  void open();
  void close();
  bool isOpen() const;

  void bind();

  /*
   * Blocking Send Operation
   */
  std::size_t send(const MutSocketBuffer &buff);

  /*
   * Blocking Receive Operation
   */
  size_t receive(const MutSocketBuffer &buff);

  /*
   * NonBlocking Send Operation
   */
  void asyncSend(const MutSocketBuffer &buff);

  /*
   * NonBlocking Receive Operation
   */
  void asyncReceive(Functor func);

private:
  void asyncSendHandler(const boost::system::error_code &error, std::size_t bytes_transferred);
  void asyncReceiveHandler(const boost::system::error_code &error, std::size_t bytes_transferred);

private:
  const IoContext &m_ctx;
  udp::socket m_udp_socket;
  udp::endpoint m_endpoint;
  Functor m_func;
  static const size_t m_recv_buffer_size{2048};
  boost::array<uint8_t, m_recv_buffer_size> m_recv_buffer;
};

}  // namespace drivers
}  // namespace autoware

#endif //UDP_DRIVER_UDP_SOCKET_HPP
