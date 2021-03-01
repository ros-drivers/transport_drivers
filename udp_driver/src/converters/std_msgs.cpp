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

#include "converters/std_msgs.hpp"

namespace autoware {
namespace msgs {

/*
 * ROS2 Message to Raw Buffer Converters
 * std_msgs::msg::Int variant
 */
void convertFromRos2Message(const std_msgs::msg::Int8::SharedPtr &in, MutSocketBuffer &out) {
  out = MutSocketBuffer(&in->data, sizeof(in->data));
}

void convertFromRos2Message(const std_msgs::msg::Int16::SharedPtr &in, MutSocketBuffer &out) {
  out = MutSocketBuffer(&in->data, sizeof(in->data));
}

void convertFromRos2Message(const std_msgs::msg::Int32::SharedPtr &in, MutSocketBuffer &out) {
  out = MutSocketBuffer(&in->data, sizeof(in->data));
}

void convertFromRos2Message(const std_msgs::msg::Int64::SharedPtr &in, MutSocketBuffer &out) {
  out = MutSocketBuffer(&in->data, sizeof(in->data));
}

/*
 * Raw Buffer to ROS2 Message Converters
 * std_msgs::msg::Int variant
 */
void convertToRos2Message(const MutSocketBuffer &in, std_msgs::msg::Int8 &out) {
  out.data = *boost::asio::buffer_cast<int8_t *>(in);
}

void convertToRos2Message(const MutSocketBuffer &in, std_msgs::msg::Int16 &out) {
  out.data = *boost::asio::buffer_cast<int16_t *>(in);
}

void convertToRos2Message(const MutSocketBuffer &in, std_msgs::msg::Int32 &out) {
  out.data = *boost::asio::buffer_cast<int32_t *>(in);
}

void convertToRos2Message(const MutSocketBuffer &in, std_msgs::msg::Int64 &out) {
  out.data = *boost::asio::buffer_cast<int64_t *>(in);
}

/*
 * ROS2 Message to Raw Buffer Converters
 * std_msgs::msg::UInt variant
 */
void convertFromRos2Message(const std_msgs::msg::UInt8::SharedPtr &in, MutSocketBuffer &out) {
  out = MutSocketBuffer(&in->data, sizeof(in->data));
}

void convertFromRos2Message(const std_msgs::msg::UInt16::SharedPtr &in, MutSocketBuffer &out) {
  out = MutSocketBuffer(&in->data, sizeof(in->data));
}

void convertFromRos2Message(const std_msgs::msg::UInt32::SharedPtr &in, MutSocketBuffer &out) {
  out = MutSocketBuffer(&in->data, sizeof(in->data));
}

void convertFromRos2Message(const std_msgs::msg::UInt64::SharedPtr &in, MutSocketBuffer &out) {
  out = MutSocketBuffer(&in->data, sizeof(in->data));
}

/*
 * Raw Buffer to ROS2 Message Converters
 * std_msgs::msg::UInt variant
 */
void convertToRos2Message(const MutSocketBuffer &in, std_msgs::msg::UInt8 &out) {
  out.data = *boost::asio::buffer_cast<int8_t *>(in);
}

void convertToRos2Message(const MutSocketBuffer &in, std_msgs::msg::UInt16 &out) {
  out.data = *boost::asio::buffer_cast<int16_t *>(in);
}

void convertToRos2Message(const MutSocketBuffer &in, std_msgs::msg::UInt32 &out) {
  out.data = *boost::asio::buffer_cast<int32_t *>(in);
}

void convertToRos2Message(const MutSocketBuffer &in, std_msgs::msg::UInt64 &out) {
  out.data = *boost::asio::buffer_cast<int64_t *>(in);
}

/*
 * ROS2 Message to Raw Buffer Converters
 * std_msgs::msg::Float variant
 */
void convertFromRos2Message(const std_msgs::msg::Float32::SharedPtr &in, MutSocketBuffer &out) {
  out = MutSocketBuffer(&in->data, sizeof(in->data));
}

void convertFromRos2Message(const std_msgs::msg::Float64::SharedPtr &in, MutSocketBuffer &out) {
  out = MutSocketBuffer(&in->data, sizeof(in->data));
}

/*
 * Raw Buffer to ROS2 Message Converters
 * std_msgs::msg::Float variant
 */
void convertToRos2Message(const MutSocketBuffer &in, std_msgs::msg::Float32 &out) {
  out.data = *boost::asio::buffer_cast<float_t *>(in);
}

void convertToRos2Message(const MutSocketBuffer &in, std_msgs::msg::Float64 &out) {
  out.data = *boost::asio::buffer_cast<double_t *>(in);
}

}
}

