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

#include "udp_driver/udp_driver.hpp"

#include <iostream>
#include <string>
#include <memory>

namespace drivers
{
namespace udp_driver
{

UdpDriver::UdpDriver(const IoContext & ctx)
: m_ctx(ctx)
{
}

void UdpDriver::init_sender(const std::string & ip, uint16_t port)
{
  m_sender.reset(new UdpSocket(m_ctx, ip, port));
}

void UdpDriver::init_sender(
  const std::string & remote_ip, uint16_t remote_port,
  const std::string & host_ip, uint16_t host_port)
{
  m_sender.reset(new UdpSocket(m_ctx, remote_ip, remote_port, host_ip, host_port));
}

void UdpDriver::init_receiver(const std::string & ip, uint16_t port)
{
  m_receiver.reset(new UdpSocket(m_ctx, ip, port));
}

std::shared_ptr<UdpSocket> UdpDriver::sender() const
{
  return m_sender;
}

std::shared_ptr<UdpSocket> UdpDriver::receiver() const
{
  return m_receiver;
}

}  // namespace udp_driver
}  // namespace drivers
