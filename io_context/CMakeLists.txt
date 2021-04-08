# Copyright 2021 Apex.AI, Inc.
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# Co-developed by Tier IV, Inc. and Apex.AI, Inc.
# Maintained by LeoDrive, 2021

cmake_minimum_required(VERSION 3.5)
project(io_context)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

## dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(asio_cmake_module REQUIRED)
find_package(ASIO REQUIRED)

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/io_context.cpp
  src/msg_converters/std_msgs.cpp
)
ament_target_dependencies(${PROJECT_NAME} "ASIO")

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  get_filename_component(name test_io_context NAME_WLE)
  set(TEST_UDP_DRIVER_EXE ${name})

  ament_add_gtest(${TEST_UDP_DRIVER_EXE}
          test/${name}.cpp)
  ament_target_dependencies(${TEST_UDP_DRIVER_EXE} std_msgs udp_msgs "ASIO")

  target_include_directories(${TEST_UDP_DRIVER_EXE} PRIVATE include)
  target_link_libraries(${TEST_UDP_DRIVER_EXE} ${PROJECT_NAME})
endif()

target_compile_options(${PROJECT_NAME} PUBLIC "-O0" PRIVATE -Wall)

ament_auto_package(
  CONFIG_EXTRAS_POST "io_context-extras.cmake"
)
