// Copyright 2021 LeoDrive, Copyright 2021 The Autoware Foundation
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

#ifndef MSG_CONVERTERS__EXAMPLE_INTERFACES_HPP_
#define MSG_CONVERTERS__EXAMPLE_INTERFACES_HPP_

#include <example_interfaces/msg/u_int8_multi_array.hpp>

#include <algorithm>
#include <vector>

#include "io_context/common.hpp"

using example_interfaces::msg::UInt8MultiArray;

namespace drivers
{
namespace common
{

inline void from_msg(const UInt8MultiArray & in, std::vector<uint8_t> & out)
{
  std::copy(in.data.begin(), in.data.end(), out.begin());
}

inline void to_msg(const std::vector<uint8_t> & in, UInt8MultiArray & out)
{
  std::copy(in.begin(), in.end(), out.data.begin());
}

}  // namespace common
}  // namespace drivers

#endif  // MSG_CONVERTERS__EXAMPLE_INTERFACES_HPP_
