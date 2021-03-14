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

#include "msg_converters/example_interfaces.hpp"

#include <vector>

using example_interfaces::msg::UInt8MultiArray;

namespace drivers
{
namespace common
{

void from_msg(const UInt8MultiArray::SharedPtr & in, MutBuffer & out)
{
  out = MutBuffer(&in->data, sizeof(in->data));
}

void to_msg(const MutBuffer & in, UInt8MultiArray & out)
{
  out.data.reserve(sizeof(in));
  out.data = *asio::buffer_cast<std::vector<uint8_t> *>(in);
}

}  // namespace common
}  // namespace drivers
