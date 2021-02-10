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

#ifndef UDP_DRIVER_UDP_SENDER_HPP
#define UDP_DRIVER_UDP_SENDER_HPP

#include <boost/asio.hpp>
#include <boost/array.hpp>

#include "visibility_control.hpp"

using boost::asio::ip::udp;
using boost::asio::ip::address;

namespace autoware
{
namespace drivers
{
/// \brief A template class and associated utilities which encapsulate basic reading and writing of UDP socket
namespace udp_driver
{

class UdpSender {
public:
    UdpSender(std::string ip, uint16_t port):
            m_io_service(),
            m_udp_socket(m_io_service, udp::endpoint(address::from_string(ip), port)) {
        m_udp_socket.open(udp::v4());
    }

    ~UdpSender() {
        boost::system::error_code err;
        m_udp_socket.close(err);
        if (err) {
            throw boost::system::system_error(err);
        }
    }

    template <typename PacketT>
    std::size_t send(PacketT & pkt) {
        try {
            constexpr size_t max_data_size = 64 * 1024;
            return m_udp_socket.send(boost::asio::buffer(&pkt, max_data_size));
        } catch (const boost::system::system_error& ex) {
            std::cout << "[ERROR] UdpSender::send => " << ex.what() << std::endl;
            return -1;
        }
    }

private:
    boost::asio::io_service m_io_service;
    boost::asio::ip::udp::socket m_udp_socket;
};

}  // namespace udp_driver
}  // namespace drivers
}  // namespace autoware

#endif //UDP_DRIVER_UDP_SENDER_HPP
