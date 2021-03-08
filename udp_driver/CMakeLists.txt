# Copyright 2018 Apex.AI, Inc.
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# Co-developed by Tier IV, Inc. and Apex.AI, Inc.
# Maintained by LeoDrive, 2021

cmake_minimum_required(VERSION 3.5)
project(udp_driver)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Consider passing `-DCMAKE_BUILD_TYPE=Debug` option to cmake to export Debug symbols.
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3")

## dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# add package modules path, not needed in dependend packages
list(INSERT CMAKE_MODULE_PATH 0 "${CMAKE_CURRENT_SOURCE_DIR}/cmake/Modules")

find_package(ASIO REQUIRED)

ament_auto_add_library(${PROJECT_NAME} SHARED
  src/udp_socket.cpp
  src/udp_driver.cpp
)
ament_target_dependencies(${PROJECT_NAME} "ASIO")
target_compile_definitions(${PROJECT_NAME} PRIVATE
  ASIO_STANDALONE)

ament_auto_add_library(${PROJECT_NAME}_nodes SHARED
  src/udp_receiver_node.cpp
  src/udp_sender_node.cpp
)
ament_target_dependencies(${PROJECT_NAME}_nodes "Boost")
target_link_libraries(${PROJECT_NAME}_nodes ${PROJECT_NAME})

rclcpp_components_register_node(${PROJECT_NAME}_nodes
  PLUGIN "drivers::udp_driver::UdpReceiverNode"
  EXECUTABLE udp_receiver_node_exe
)

rclcpp_components_register_node(${PROJECT_NAME}_nodes
  PLUGIN "drivers::udp_driver::UdpSenderNode"
  EXECUTABLE udp_sender_node_exe
)

ament_auto_add_executable(udp_bridge_node_exe
  src/udp_bridge_node.cpp
)

target_link_libraries(udp_bridge_node_exe ${PROJECT_NAME} ${PROJECT_NAME}_nodes)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  set(TEST_FILES
    test_udp_socket.cpp
    test_udp_data.cpp
    test_udp_driver.cpp
    test_udp_driver_nodes.cpp)

  foreach(file ${TEST_FILES})
    get_filename_component(name "${file}" NAME_WE)
    set(TEST_UDP_DRIVER_EXE ${name})

    ament_add_gtest(${TEST_UDP_DRIVER_EXE}
      test/${name}.cpp)
    ament_target_dependencies(${TEST_UDP_DRIVER_EXE}
      rclcpp
      std_msgs
      lifecycle_msgs
      ASIO)

    target_include_directories(${TEST_UDP_DRIVER_EXE} PRIVATE include)
    target_link_libraries(${TEST_UDP_DRIVER_EXE} ${PROJECT_NAME} ${PROJECT_NAME}_nodes)
  endforeach()
endif()

ament_auto_package()
