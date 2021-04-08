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

#ifndef SERIAL_DRIVER__SERIAL_DRIVER_HPP_
#define SERIAL_DRIVER__SERIAL_DRIVER_HPP_

#include <memory>
#include <string>

#include "io_context/io_context.hpp"
#include "serial_driver/serial_port.hpp"

namespace drivers
{
namespace serial_driver
{

class SerialDriver
{
public:
  explicit SerialDriver(const IoContext & ctx);

  void init_port(const std::string & device_name, const SerialPortConfig & config);

  std::shared_ptr<SerialPort> port() const;

private:
  const IoContext & m_ctx;
  std::shared_ptr<SerialPort> m_port;
};

}  // namespace serial_driver
}  // namespace drivers

#endif  // SERIAL_DRIVER__SERIAL_DRIVER_HPP_
