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

#include <gtest/gtest.h>

#include <vector>
#include <memory>

#include "io_context/io_context.hpp"

using drivers::common::IoContext;

static constexpr size_t LENGTH = 10;

void counter(std::shared_ptr<std::vector<int32_t>> container, int32_t count)
{
  container->push_back(count);
}

void check_container_size(std::shared_ptr<std::vector<int32_t>> container)
{
  EXPECT_EQ(container->size(), LENGTH);
}

TEST(IoContextTest, DefaultLifeCycleTest)
{
  IoContext ctx;
  EXPECT_EQ(ctx.isServiceStopped(), false);
  EXPECT_EQ(ctx.serviceThreadCount(), std::thread::hardware_concurrency());
}

TEST(IoContextTest, ConcurrentLifeCycleTest)
{
  IoContext ctx(LENGTH);
  EXPECT_EQ(ctx.isServiceStopped(), false);
  EXPECT_EQ(ctx.serviceThreadCount(), LENGTH);
}

TEST(IoContextTest, SingleThreadPostTaskTest)
{
  IoContext ctx(1);
  EXPECT_EQ(ctx.isServiceStopped(), false);
  EXPECT_EQ(ctx.serviceThreadCount(), uint32_t(1));

  std::shared_ptr<std::vector<int32_t>> container(new std::vector<int32_t>());
  for (size_t i = 0; i < LENGTH; ++i) {
    ctx.post(std::bind(counter, container, i));
  }
  ctx.post(std::bind(check_container_size, container));
}
