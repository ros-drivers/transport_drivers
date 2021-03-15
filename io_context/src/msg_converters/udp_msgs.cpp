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
#include <vector>

#include "msg_converters/udp_msgs.hpp"


namespace drivers
{
namespace common
{

/*
 * ROS2 Message to Raw Buffer Converters
 * udp_msgs::msg::UdpPacket variant
 */
void from_msg(const udp_msgs::msg::UdpPacket & in, MutBuffer & out)
{
  out = MutBuffer(const_cast<std::vector<uint8_t> *>(&in.data), sizeof(in.data));
}

void to_msg(const MutBuffer & in, udp_msgs::msg::UdpPacket & out)
{
  out.data.reserve(sizeof(in));
  out.data = *asio::buffer_cast<std::vector<uint8_t> *>(in);
}

}  // namespace common
}  // namespace drivers
