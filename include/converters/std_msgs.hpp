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

#ifndef TRANSPORT_DRIVER_STD_MSGS_HPP
#define TRANSPORT_DRIVER_STD_MSGS_HPP

#include <std_msgs/msg/int8.hpp>
#include <std_msgs/msg/int16.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/int64.hpp>

#include "common.hpp"

namespace autoware {
namespace msgs {

/*
* ROS2 Message to Raw Buffer Converters
*/
void convertFromRos2Message(const std_msgs::msg::Int8::SharedPtr &in, MutSocketBuffer &out);
void convertFromRos2Message(const std_msgs::msg::Int16::SharedPtr &in, MutSocketBuffer &out);
void convertFromRos2Message(const std_msgs::msg::Int32::SharedPtr &in, MutSocketBuffer &out);
void convertFromRos2Message(const std_msgs::msg::Int64::SharedPtr &in, MutSocketBuffer &out);

/*
* Raw Buffer to ROS2 Message Converters
*/
void convertToRos2Message(const MutSocketBuffer &in, std_msgs::msg::Int8 &out);
void convertToRos2Message(const MutSocketBuffer &in, std_msgs::msg::Int16 &out);
void convertToRos2Message(const MutSocketBuffer &in, std_msgs::msg::Int32 &out);
void convertToRos2Message(const MutSocketBuffer &in, std_msgs::msg::Int64 &out);

}  // namespace msgs
}  // namespace autoware

#endif //TRANSPORT_DRIVER_STD_MSGS_HPP
