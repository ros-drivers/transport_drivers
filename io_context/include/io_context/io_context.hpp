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

#ifndef IO_CONTEXT__IO_CONTEXT_HPP_
#define IO_CONTEXT__IO_CONTEXT_HPP_

#include <boost/asio.hpp>
#include <boost/thread.hpp>
#include <boost/noncopyable.hpp>

#include <memory>

namespace drivers
{

class IoContext : private boost::noncopyable
{
public:
  explicit IoContext(size_t threads_count = -1);
  ~IoContext();

  boost::asio::io_service & ios() const;

  bool isServiceStopped();
  uint32_t serviceThreadCount();

  void waitForExit();

  template<class F>
  void post(F f)
  {
    ios().post(f);
  }

private:
  std::shared_ptr<boost::asio::io_service> m_ios;
  std::shared_ptr<boost::asio::io_service::work> m_work;
  std::shared_ptr<boost::thread_group> m_ios_thread_workers;
};

}  // namespace drivers

#endif  // IO_CONTEXT__IO_CONTEXT_HPP_
