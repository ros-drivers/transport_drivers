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

#ifndef UDP_DRIVER_CONVERTERS_HPP
#define UDP_DRIVER_CONVERTERS_HPP

#include <boost/asio.hpp>
#include "std_msgs/msg/int32.hpp"

typedef boost::asio::mutable_buffer MutSocketBuffer;

namespace autoware
{
namespace msgs
{

inline void convertFromRosMessage(const std_msgs::msg::Int32::SharedPtr &in, MutSocketBuffer &out) {
  out = MutSocketBuffer(&in->data, sizeof(in->data));
}

inline void convertToRosMessage(const MutSocketBuffer &in, std_msgs::msg::Int32 &out) {
  out.data = *boost::asio::buffer_cast<int32_t *>(in);
}

}  // namespace msgs
}  // namespace autoware

#endif //UDP_DRIVER_CONVERTERS_HPP
