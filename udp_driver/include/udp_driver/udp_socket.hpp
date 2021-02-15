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
#include <boost/asio.hpp>
#include <boost/array.hpp>
#include "io_context.hpp"
#include "converters.hpp"

using boost::asio::ip::udp;
using boost::asio::ip::address;

namespace autoware
{
namespace drivers
{
namespace udp_driver
{

class UdpSocket : private boost::noncopyable {
public:
    UdpSocket(const IoContext &ctx, const std::string &ip, uint16_t port): m_ctx(ctx),
        m_endpoint(address::from_string(ip), port),
        m_udp_socket(ctx.ios()) {
    }

    ~UdpSocket() {
        std::cout << "[UdpSocket::~UdpSocket] INFO => Destructing..." << std::endl;
        close();
    }

    inline std::string ip() const { return m_endpoint.address().to_string(); }
    inline uint16_t port() const { return m_endpoint.port(); }
    inline void open() { return m_udp_socket.open(udp::v4()); }
    inline bool isOpen() const { return m_udp_socket.is_open(); }
    inline void bind() { return m_udp_socket.bind(m_endpoint); }

    std::size_t send(const MutSocketBuffer &buff) {
        try {
            return m_udp_socket.send_to(buff, m_endpoint);
        } catch (const boost::system::system_error& error) {
            std::cout << "[UdpSocket::send] Error => " << error.what() << std::endl;
            return -1;
        }
    }

    size_t receive(const MutSocketBuffer &buff) {
        boost::system::error_code error;
        boost::asio::ip::udp::endpoint sender_endpoint;

        std::size_t  len = m_udp_socket.receive_from(buff,m_endpoint,0,error);

        if (error && error != boost::asio::error::message_size) {
            std::cout << "[UdpSocket::receive] Error => " << error.message() << std::endl;
            return -1;
        }
        return len;
    }

    void asyncSend(const MutSocketBuffer &buff) {
        m_udp_socket.async_send_to(buff, m_endpoint,boost::bind(
                &UdpSocket::asyncSendHandler,
                this,
                boost::asio::placeholders::error,
                boost::asio::placeholders::bytes_transferred));
    }

    template<typename F>
    void asyncReceive(F func) {
        m_func = func;
        m_udp_socket.async_receive_from(
                boost::asio::buffer(m_recv_buffer, m_recv_buffer_size),
                m_endpoint,
                boost::bind(&UdpSocket::asyncReceiveHandler, this,
                            boost::asio::placeholders::error,
                            boost::asio::placeholders::bytes_transferred));
    }

    void close() {
        boost::system::error_code error;
        m_udp_socket.close(error);
        if (error) {
            std::cout << "[UdpSocket::close] Error => " << error.message() << std::endl;
        }
    }

    void asyncSendHandler(const boost::system::error_code& error, std::size_t bytes_transferred) {
        if (error) {
            std::cout << "[UdpSocket::asyncSendHandler] Error => " << error.message() << std::endl;
        }
    }

    void asyncReceiveHandler(const boost::system::error_code& error, std::size_t bytes_transferred) {
        if (error) {
            std::cout << "[UdpSocket::asyncReceiveHandler] Error => " << error.message() << std::endl;
            return;
        }

        if (bytes_transferred > 0 && m_func) {
            m_func(MutSocketBuffer(m_recv_buffer.data(), bytes_transferred));
            m_udp_socket.async_receive_from(
                    boost::asio::buffer(m_recv_buffer, m_recv_buffer_size),
                    m_endpoint,
                    boost::bind(&UdpSocket::asyncReceiveHandler, this,
                                boost::asio::placeholders::error,
                                boost::asio::placeholders::bytes_transferred));
        }
    }

private:
    const IoContext &m_ctx;
    udp::socket m_udp_socket;
    udp::endpoint m_endpoint;
    boost::function<void(const MutSocketBuffer &)> m_func;
    static const size_t m_recv_buffer_size {2048};
    boost::array<uint8_t, m_recv_buffer_size> m_recv_buffer;
};

}  // namespace udp_driver
}  // namespace drivers
}  // namespace autoware

#endif //UDP_DRIVER_UDP_SOCKET_HPP
