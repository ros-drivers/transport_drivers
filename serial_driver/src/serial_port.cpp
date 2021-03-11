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

#include "serial_driver/serial_port.hpp"

#include <rclcpp/logging.hpp>

#include <string>
#include <utility>

namespace drivers
{
namespace serial_driver
{

SerialPort::SerialPort(
  const IoContext & ctx,
  const std::string & device_name,
  const SerialPortConfig serial_port_config)
: m_ctx(ctx),
  m_device_name(device_name),
  m_serial_port(ctx.ios()),
  m_port_config(serial_port_config)
{
}

SerialPort::~SerialPort()
{
  close();
}

size_t SerialPort::send(const MutBuffer & buff)
{
  try {
    return m_serial_port.write_some(buff);
  } catch (const std::system_error & ex) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("SerialPort::send"), ex.what());
    return -1;
  }
}

size_t SerialPort::receive(const MutBuffer & buff)
{
  try {
    return m_serial_port.read_some(buff);
  } catch (const std::system_error & ex) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("SerialPort::receive"), ex.what());
    return -1;
  }
}

void SerialPort::async_send(const MutBuffer & buff)
{
  m_serial_port.async_write_some(
    buff,
    [this](std::error_code error, size_t bytes_transferred)
    {
      async_send_handler(error, bytes_transferred);
    });
}

void SerialPort::async_receive(Functor func)
{
  m_func = std::move(func);
  m_serial_port.async_read_some(
    asio::buffer(m_recv_buffer, m_recv_buffer_size),
    [this](std::error_code error, size_t bytes_transferred)
    {
      async_receive_handler(error, bytes_transferred);
    });
}

void SerialPort::async_send_handler(
  const asio::error_code & error,
  size_t bytes_transferred)
{
  (void)bytes_transferred;
  if (error) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("SerialPort::async_send_handler"), error.message());
    return;
  }
}

void SerialPort::async_receive_handler(
  const asio::error_code & error,
  size_t bytes_transferred)
{
  if (error) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("SerialPort::async_receive_handler"), error.message());
    return;
  }

  if (bytes_transferred > 0 && m_func) {
    m_func(MutBuffer(m_recv_buffer.data(), bytes_transferred));
    m_serial_port.async_read_some(
      asio::buffer(m_recv_buffer, m_recv_buffer_size),
      [this](std::error_code error, size_t bytes_transferred)
      {
        async_receive_handler(error, bytes_transferred);
      });
  }
}

std::string SerialPort::device_name() const
{
  return m_device_name;
}

SerialPortConfig SerialPort::serial_port_config() const
{
  return m_port_config;
}

void SerialPort::open()
{
  m_serial_port.open(m_device_name);
  m_serial_port.set_option(sbp::baud_rate(m_port_config.get_baud_rate_asio()));
  m_serial_port.set_option(sbp::flow_control(m_port_config.get_flow_control_asio()));
  m_serial_port.set_option(sbp::parity(m_port_config.get_parity_asio()));
  m_serial_port.set_option(sbp::stop_bits(m_port_config.get_stop_bits_asio()));
}

void SerialPort::close()
{
  asio::error_code error;
  m_serial_port.close(error);
  if (error) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("SerialPort::close"), error.message());
  }
}

bool SerialPort::is_open() const
{
  return m_serial_port.is_open();
}

}  // namespace serial_driver
}  // namespace drivers
