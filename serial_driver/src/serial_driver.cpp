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

#include "serial_driver/serial_driver.hpp"

#include <memory>
#include <string>

namespace drivers
{
namespace serial_driver
{

SerialDriver::SerialDriver(const IoContext & ctx)
: m_ctx(ctx)
{
}

void SerialDriver::init_port(
  const std::string & device_name,
  const SerialPortConfig & config)
{
  m_port.reset(new SerialPort(m_ctx, device_name, config));
}

std::shared_ptr<SerialPort> SerialDriver::port() const
{
  return m_port;
}

}  // namespace serial_driver
}  // namespace drivers
