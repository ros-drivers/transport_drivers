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

#include "io_context.hpp"

#include <iostream>

namespace autoware {
namespace drivers {

IoContext::IoContext(int16_t threads_count) : m_ios(new boost::asio::io_service()),
   m_work(new boost::asio::io_service::work(ios())),
   m_ios_thread_workers(new boost::thread_group()) {
    if (threads_count == -1) {
        threads_count = boost::thread::hardware_concurrency();
    }

    for (int i = 0; i < threads_count; ++i) {
        m_ios_thread_workers->create_thread(boost::bind(&boost::asio::io_service::run, &ios()));
    }

    std::cout << "[IoContext::IoContext] INFO => Thread(s) Created: " << serviceThreadCount() << std::endl;
}

IoContext::~IoContext() {
    std::cout << "[IoContext::~IoContext] INFO => Destructing..." << std::endl;
    waitForExit();
}

boost::asio::io_service &IoContext::ios() const {
    return *m_ios;
}

bool IoContext::isServiceStopped() {
    return ios().stopped();
}

uint32_t IoContext::serviceThreadCount() {
    return m_ios_thread_workers->size();
}

void IoContext::waitForExit() {
    if (!ios().stopped()) {
        ios().post([&]() { m_work.reset(); });
    }

    m_ios_thread_workers->interrupt_all();
    m_ios_thread_workers->join_all();
}

}  // namespace drivers
}  // namespace autoware