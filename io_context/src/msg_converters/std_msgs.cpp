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

#include "msg_converters/std_msgs.hpp"

#include <cmath>
#include <vector>

#include "asio.hpp"

namespace drivers
{
namespace common
{

/*
 * ROS2 Message to Raw Buffer Converters
 * std_msgs::msg::Int variant
 */
void from_msg(const std_msgs::msg::Int8::SharedPtr & in, std::vector<uint8_t> & out)
{
  out.resize(sizeof(in->data));
  std::memcpy(&out[0], &in->data, sizeof(in->data));
}

void from_msg(const std_msgs::msg::Int16::SharedPtr & in, std::vector<uint8_t> & out)
{
  out.resize(sizeof(in->data));
  std::memcpy(&out[0], &in->data, sizeof(in->data));
}

void from_msg(const std_msgs::msg::Int32::SharedPtr & in, std::vector<uint8_t> & out)
{
  out.resize(sizeof(in->data));
  std::memcpy(&out[0], &in->data, sizeof(in->data));
}

void from_msg(const std_msgs::msg::Int64::SharedPtr & in, std::vector<uint8_t> & out)
{
  out.resize(sizeof(in->data));
  std::memcpy(&out[0], &in->data, sizeof(in->data));
}

/*
 * ROS2 Message to Raw Buffer Converters
 * std_msgs::msg::UInt variant
 */
void from_msg(const std_msgs::msg::UInt8::SharedPtr & in, std::vector<uint8_t> & out)
{
  out.resize(sizeof(in->data));
  std::memcpy(&out[0], &in->data, sizeof(in->data));
}

void from_msg(const std_msgs::msg::UInt16::SharedPtr & in, std::vector<uint8_t> & out)
{
  out.resize(sizeof(in->data));
  std::memcpy(&out[0], &in->data, sizeof(in->data));
}

void from_msg(const std_msgs::msg::UInt32::SharedPtr & in, std::vector<uint8_t> & out)
{
  out.resize(sizeof(in->data));
  std::memcpy(&out[0], &in->data, sizeof(in->data));
}

void from_msg(const std_msgs::msg::UInt64::SharedPtr & in, std::vector<uint8_t> & out)
{
  out.resize(sizeof(in->data));
  std::memcpy(&out[0], &in->data, sizeof(in->data));
}

/*
 * ROS2 Message to Raw Buffer Converters
 * std_msgs::msg::Float variant
 */
void from_msg(const std_msgs::msg::Float32::SharedPtr & in, std::vector<uint8_t> & out)
{
  out.resize(sizeof(in->data));
  std::memcpy(&out[0], &in->data, sizeof(in->data));
}

void from_msg(const std_msgs::msg::Float64::SharedPtr & in, std::vector<uint8_t> & out)
{
  out.resize(sizeof(in->data));
  std::memcpy(&out[0], &in->data, sizeof(in->data));
}

/*
 * Raw Buffer to ROS2 Message Converters
 * std_msgs::msg::Int variant
 */
void to_msg(const std::vector<uint8_t> & in, std_msgs::msg::Int8 & out)
{
  out.data = *reinterpret_cast<int8_t *>(in[0]);
}

void to_msg(const std::vector<int16_t> & in, std_msgs::msg::Int16 & out)
{
  out.data = *reinterpret_cast<int16_t *>(in[0]);
}

void to_msg(const std::vector<uint8_t> & in, std_msgs::msg::Int32 & out)
{
  out.data = *reinterpret_cast<int32_t *>(in[0]);
}

void to_msg(const std::vector<uint8_t> & in, std_msgs::msg::Int64 & out)
{
  out.data = *reinterpret_cast<int64_t *>(in[0]);
}

/*
 * Raw Buffer to ROS2 Message Converters
 * std_msgs::msg::UInt variant
 */
void to_msg(const std::vector<uint8_t> & in, std_msgs::msg::UInt8 & out)
{
  out.data = *reinterpret_cast<uint8_t *>(in[0]);
}

void to_msg(const std::vector<uint8_t> & in, std_msgs::msg::UInt16 & out)
{
  out.data = *reinterpret_cast<uint16_t *>(in[0]);
}

void to_msg(const std::vector<uint8_t> & in, std_msgs::msg::UInt32 & out)
{
  out.data = *reinterpret_cast<uint32_t *>(in[0]);
}

void to_msg(const std::vector<uint8_t> & in, std_msgs::msg::UInt64 & out)
{
  out.data = *reinterpret_cast<uint64_t *>(in[0]);
}

/*
 * Raw Buffer to ROS2 Message Converters
 * std_msgs::msg::Float variant
 */
void to_msg(const std::vector<uint8_t> & in, std_msgs::msg::Float32 & out)
{
  out.data = *reinterpret_cast<float *>(in[0]);
}

void to_msg(const std::vector<uint8_t> & in, std_msgs::msg::Float64 & out)
{
  out.data = *reinterpret_cast<double *>(in[0]);
}

}  // namespace common
}  // namespace drivers
