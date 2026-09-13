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

cmake_minimum_required(VERSION 3.14.4)
project(io_context)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)

## dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(asio_cmake_module REQUIRED)
find_package(ASIO REQUIRED)

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/io_context.cpp
  src/msg_converters/std_msgs.cpp
)
target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC ${ASIO_INCLUDE_DIRS})
target_compile_definitions(${PROJECT_NAME} PUBLIC ${ASIO_DEFINITIONS})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  get_filename_component(name test_io_context NAME_WLE)
  set(TEST_UDP_DRIVER_EXE ${name})

  ament_add_gtest(${TEST_UDP_DRIVER_EXE}
          test/${name}.cpp)
  target_link_libraries(${TEST_UDP_DRIVER_EXE}
      ${std_msgs_TARGETS}
      ${udp_msgs_TARGETS}
  )
  target_include_directories(${TEST_UDP_DRIVER_EXE} SYSTEM PUBLIC ${ASIO_INCLUDE_DIRS})
  target_compile_definitions(${TEST_UDP_DRIVER_EXE} PUBLIC ${ASIO_DEFINITIONS})

  target_include_directories(${TEST_UDP_DRIVER_EXE} PRIVATE include)
  target_link_libraries(${TEST_UDP_DRIVER_EXE} ${PROJECT_NAME})
endif()

target_compile_options(${PROJECT_NAME} PUBLIC "-O0" PRIVATE -Wall)

ament_auto_package(
  CONFIG_EXTRAS_POST "io_context-extras.cmake"
)
