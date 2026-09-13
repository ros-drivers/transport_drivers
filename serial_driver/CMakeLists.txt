# Copyright 2018 Apex.AI, Inc.
#
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
#
# Co-developed by Tier IV, Inc. and Apex.AI, Inc.
cmake_minimum_required(VERSION 3.14.4)
project(serial_driver)

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
  src/serial_port.cpp
  src/serial_driver.cpp
)

target_include_directories(${PROJECT_NAME} SYSTEM PUBLIC ${ASIO_INCLUDE_DIRS})
target_compile_definitions(${PROJECT_NAME} PUBLIC ${ASIO_DEFINITIONS})

ament_auto_add_library(${PROJECT_NAME}_nodes SHARED
  src/serial_bridge_node.cpp
)

rclcpp_components_register_node(${PROJECT_NAME}_nodes
  PLUGIN "drivers::serial_driver::SerialBridgeNode"
  EXECUTABLE "serial_bridge"
)

target_link_libraries(${PROJECT_NAME}_nodes ${PROJECT_NAME})
target_include_directories(${PROJECT_NAME}_nodes SYSTEM PUBLIC ${ASIO_INCLUDE_DIRS})
target_compile_definitions(${PROJECT_NAME}_nodes PUBLIC ${ASIO_DEFINITIONS})

if(BUILD_TESTING)
  find_package(ament_lint_auto)
  ament_lint_auto_find_test_dependencies()

  set(TEST_FILES
    test/test_serial_driver.cpp
    test/test_serial_port.cpp
  )

  foreach(file ${TEST_FILES})
    get_filename_component(name "${file}" NAME_WE)
    set(TEST_SERIAL_DRIVER_EXE ${name})

    ament_add_gtest(${TEST_SERIAL_DRIVER_EXE}
      test/${name}.cpp
    )
    target_link_libraries(${TEST_SERIAL_DRIVER_EXE}
        ${rclcpp_TARGETS}
        ${lifecycle_msgs_TARGETS}
    )
    target_include_directories(${TEST_SERIAL_DRIVER_EXE} SYSTEM PUBLIC ${ASIO_INCLUDE_DIRS})
    target_compile_definitions(${TEST_SERIAL_DRIVER_EXE} PUBLIC ${ASIO_DEFINITIONS})
    target_include_directories(${TEST_SERIAL_DRIVER_EXE} PRIVATE include)
    target_link_libraries(${TEST_SERIAL_DRIVER_EXE} ${PROJECT_NAME} ${PROJECT_NAME_NODES})
  endforeach()
endif()

ament_auto_package(
  CONFIG_EXTRAS_POST "serial_driver-extras.cmake"
  INSTALL_TO_SHARE
    launch
    params
)
