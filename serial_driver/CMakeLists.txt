# Copyright 2018 Apex.AI, Inc.
# Co-developed by Tier IV, Inc. and Apex.AI, Inc.
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
cmake_minimum_required(VERSION 3.5)

project(serial_driver)

## dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(autoware_auto_cmake REQUIRED)
find_package(autoware_auto_helper_functions REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
if(Boost_FOUND)
    include_directories(${Boost_INCLUDE_DIRS})
    link_directories(${Boost_LIBRARY_DIRS})
endif()

include_directories(include)

ament_export_dependencies("autoware_auto_helper_functions")
autoware_install(HAS_INCLUDE)

if(BUILD_TESTING)
    find_package(std_msgs REQUIRED)
    set(TEST_LIB "${PROJECT_NAME}_test")
    add_library(${TEST_LIB} SHARED test/test_driver.hpp test/test_driver.cpp)

    ament_target_dependencies(${TEST_LIB} "autoware_auto_helper_functions" "rclcpp" "rclcpp_lifecycle" "std_msgs")
    target_include_directories(${TEST_LIB} PRIVATE include)
    target_link_libraries(${TEST_LIB} ${Boost_LIBRARIES})

    # Unit tests
    find_package(ament_cmake_gtest REQUIRED)
    set(TEST_SERIAL_DRIVER_EXE test_serial_driver)
    ament_add_gtest(${TEST_SERIAL_DRIVER_EXE}
            test/gtest_main.cpp
            test/test_serial_driver.cpp)
    ament_target_dependencies(${TEST_SERIAL_DRIVER_EXE}  "Boost")
    target_include_directories(${TEST_SERIAL_DRIVER_EXE} PRIVATE include)
    target_link_libraries(${TEST_SERIAL_DRIVER_EXE} ${TEST_LIB} util)

    autoware_static_code_analysis()
endif()

ament_package(
    CONFIG_EXTRAS_POST "serial_driver-extras.cmake"
)
