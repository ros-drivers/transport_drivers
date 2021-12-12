// Copyright 2021 Evan Flynn.
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

// Developed by Evan Flynn, 2021

#ifndef MSG_CONVERTERS__UDP_MSGS_HPP_
#define MSG_CONVERTERS__UDP_MSGS_HPP_

#include <udp_msgs/msg/udp_packet.hpp>

#include <vector>
#include <algorithm>


namespace drivers
{
namespace common
{

/*
 * ROS2 Message to Raw Buffer Converters
 * udp_msgs::msg::UdpPacket variant
 */
inline void from_msg(const udp_msgs::msg::UdpPacket::SharedPtr & in, std::vector<uint8_t> & out)
{
  out.resize(sizeof(in->data));
  std::copy(in->data.begin(), in->data.end(), out.begin());
}

inline void to_msg(const std::vector<uint8_t> & in, udp_msgs::msg::UdpPacket & out)
{
  out.data.resize(sizeof(in));
  std::copy(in.begin(), in.end(), out.data.begin());
}

}  // namespace common
}  // namespace drivers

#endif  // MSG_CONVERTERS__UDP_MSGS_HPP_
