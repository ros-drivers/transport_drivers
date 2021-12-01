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

#include <memory>
#include <vector>
#include <utility>

#include "io_context/common.hpp"

namespace drivers
{
namespace common
{

//! A workaround of boost::thread_group
// Copied from https://gist.github.com/coin-au-carre/ceb8a790cec3b3535b015be3ec2a1ce2
struct thread_group
{
  std::vector<std::thread> tg;

  thread_group()                                  = default;
  thread_group(const thread_group &)               = delete;
  thread_group & operator=(const thread_group &)    = delete;
  thread_group(thread_group &&)                   = delete;

  template<class ... Args>
  void create_thread(Args && ... args) {tg.emplace_back(std::forward<Args>(args)...);}

  void add_thread(std::thread && t) {tg.emplace_back(std::move(t));}

  std::size_t size() const {return tg.size();}

  void join_all()
  {
    for (auto & thread : tg) {
      if (thread.joinable()) {
        thread.join();
      }
    }
  }
};

class IoContext
{
public:
  IoContext();
  explicit IoContext(size_t threads_count);
  ~IoContext();

  IoContext(const IoContext &) = delete;
  IoContext & operator=(const IoContext &) = delete;

  asio::io_service & ios() const;

  bool isServiceStopped();
  uint32_t serviceThreadCount();

  void waitForExit();

  template<class F>
  void post(F f)
  {
    ios().post(f);
  }

private:
  std::shared_ptr<asio::io_service> m_ios;
  std::shared_ptr<asio::io_service::work> m_work;
  std::shared_ptr<drivers::common::thread_group> m_ios_thread_workers;
};

}  // namespace common
}  // namespace drivers

#endif  // IO_CONTEXT__IO_CONTEXT_HPP_
