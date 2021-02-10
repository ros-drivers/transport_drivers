// Copyright 2018 Apex.AI, Inc.
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

// Co-developed by Tier IV, Inc. and Apex.AI, Inc.
// Maintained by LeoDrive, 2021

#ifndef UDP_DRIVER_UDP_RECEIVER_HPP
#define UDP_DRIVER_UDP_RECEIVER_HPP

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

class UdpReceiver {
public:
    UdpReceiver(std::string ip, uint16_t port) :
            m_io_service(),
            m_udp_socket(m_io_service, udp::endpoint(address::from_string(ip), port)) {
    }

    /// \brief Receives a package via UDP and returns the length of the data in the buffer
    template <typename PacketT>
    size_t receive_packet(PacketT & pkt)
    {
        boost::system::error_code udp_error;
        boost::asio::ip::udp::endpoint sender_endpoint;
        constexpr size_t max_data_size = 64 * 1024;

        const size_t len = m_udp_socket.receive_from(
                boost::asio::buffer(&pkt, max_data_size),
                sender_endpoint,
                0,
                udp_error);

        if (udp_error && udp_error != boost::asio::error::message_size) {
            throw boost::system::system_error(udp_error);
        }

        return len;
    }

private:
    boost::asio::io_service m_io_service;
    boost::asio::ip::udp::socket m_udp_socket;
};

}  // namespace udp_driver
}  // namespace drivers
}  // namespace autoware

#endif //UDP_DRIVER_UDP_RECEIVER_HPP

