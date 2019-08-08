// Copyright 2018 Apex.AI, Inc.
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

// Co-developed by Tier IV, Inc. and Apex.AI, Inc.

#include <boost/array.hpp>
#include <boost/asio.hpp>
#include <string>
#include <thread>
#include <vector>
#include "gtest/gtest.h"
#include "test_driver.hpp"


using test_udp_driver::TestDriver;
using test_udp_driver::Packet;
namespace
{
class udp_driver : public ::testing::Test
{
protected:
  void start_ping(const std::vector<int> & values)
  {
    boost::asio::io_service io_service;
    boost::asio::ip::udp::socket socket(io_service);
    socket.open(boost::asio::ip::udp::v4());
    boost::system::error_code err;

    for (auto val : values) {
      Packet * pkt = new Packet(val);
      auto sent = socket.send_to(boost::asio::buffer(pkt, sizeof(*pkt)), m_ping_endpoint, 0, err);
      if (err && err != boost::asio::error::message_size) {
        std::cerr << "Could not send packet: " << err.message();
      }
    }
  }

  void init_pinger_endpoint(const std::string & ip, const uint16_t & port)
  {
    m_ping_endpoint = boost::asio::ip::udp::endpoint(boost::asio::ip::address::from_string(
          ip), static_cast<uint16_t>(port));
  }

  boost::asio::ip::udp::endpoint m_ping_endpoint;
};
}  // namespace


// tests udp_driver_node's get_packet function which receives udp packages
TEST_F(udp_driver, basic)
{
  // rclcpp::init required to start the node
  rclcpp::init(0, nullptr);

  // setting values to send
  std::vector<int> values(10);
  std::generate(values.begin(), values.end(), [n = 0]() mutable {return n++;});

  // setting up udp_driver_node instance
  std::string ip = "127.0.0.1";
  uint16_t port = 9001;
  TestDriver driver("foo", "topic", ip, port);


  // setting up the pinger
  init_pinger_endpoint(ip, port);

  start_ping(values);


  for (auto val : values) {
    driver.run(1U);
    EXPECT_EQ(driver.get_last_value(), val);
  }
}
