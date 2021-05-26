// Copyright 2021 LeoDrive, The Autoware Foundation
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

#ifndef MSG_CONVERTERS__CONVERTERS_HPP_
#define MSG_CONVERTERS__CONVERTERS_HPP_

#include <algorithm>
#include <iostream>
#include <vector>

#include "io_context/common.hpp"

namespace drivers
{
namespace common
{

// A "basic" message is one which has a data member
// which stores the data
template<typename MsgTypeT>
inline void from_basic_msg(const typename MsgTypeT::SharedPtr & in, std::vector<uint8_t> & out)
{
  std::cout << "[ io_context::converters::from_basic_msg ] msgtype.data size: " << sizeof(in->data) << std::endl;

  out.resize(sizeof(in->data));
  std::memcpy(&out[0], &in->data, sizeof(in->data));
}

// A "basic" message is one which has a data member
// which stores the data
template<typename MsgTypeT>
inline void to_basic_msg(const std::vector<uint8_t> & in, MsgTypeT & out)
{
  using VectorTypeT = decltype(out.data);
  out.data = *reinterpret_cast<VectorTypeT *>(in[0]);
}

}  // namespace common
}  // namespace drivers

#endif  // MSG_CONVERTERS__CONVERTERS_HPP_
