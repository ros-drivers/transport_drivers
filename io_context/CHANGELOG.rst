^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package io_context
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.2.0 (2022-03-20)
------------------
* Fix linter errors.
* Contributors: WhitleySoftwareServices

1.1.0 (2022-03-20)
------------------
* Fix the converter Converter between `std_msgs::msg::UInt8MultiArray` and `std::vector<uint8_t>` (`#73 <https://github.com/ros-drivers/transport_drivers/issues/73>`_)
  * example_interfaces is redundant as std_msgs includes UInt8MultiArray
  * udp_msgs.hpp should not include "converters.hpp"
  * Fix the converter std_msgs::msg::UInt8MultiArray <-> std::vector<uint8_t>
* Add a second constructor to avoid comparing size_t (unsigned int) and int (`#70 <https://github.com/ros-drivers/transport_drivers/issues/70>`_)
* Fix cpplint error (`#69 <https://github.com/ros-drivers/transport_drivers/issues/69>`_)
  See https://build.ros2.org/job/Gdev__transport_drivers__ubuntu_focal_amd64/9/testReport/junit/(root)/projectroot/cpplint/
* Contributors: ChenJun, Esteve Fernandez

1.0.1 (2021-08-30)
------------------
* Serial driver debugs (`#56 <https://github.com/ros-drivers/transport_drivers/issues/56>`_)
  * debugged message conversion
  debugged serial bridge segfault (publisher accessed before initialized)
  * refactored message conversion
* update READMEs for each package (`#54 <https://github.com/ros-drivers/transport_drivers/issues/54>`_)
  * update READMEs for each package
  * add more to readme, renamed config to params
* use vectors not mutbuffers (`#50 <https://github.com/ros-drivers/transport_drivers/issues/50>`_)
* Port udp_driver Changes to serial_driver (`#47 <https://github.com/ros-drivers/transport_drivers/issues/47>`_)
  * Adding SerialPort and framework for SerialDriver.
  * Adding SerialDriver
  * Adding SerialBridgeNode
  * Remove serial_driver_node and tests
  * Create new tests for serial_port and serial_driver
  * Changing signature of from_msg for example_interfaces
  * [serial_driver] Adding missing function definition
  * [serial_driver] Adding example params and launch files
  * Trying to apease flake8
  * Fixing allocation snafu
  * [serial_driver] Replacing MutBuffer with std::vector<uint8_t>
  * [serial_driver] Fix typo
  * [serial_driver] Make launch file more reliable
  * [serial_driver] Fix error message
  * [serial_driver] Fix typo
  * [serial_driver] Shorten node name
* Generic udp nodes (`#40 <https://github.com/ros-drivers/transport_drivers/issues/40>`_)
  * use udp_msgs for receiver and sender nodes
  * use vector of uint8_ts instead of mutable buffer
  * all tests passing
* Rename MutSocketBuffer to MutBuffer (`#46 <https://github.com/ros-drivers/transport_drivers/issues/46>`_)
* Export ASIO definitions (`#44 <https://github.com/ros-drivers/transport_drivers/issues/44>`_)
* Enforce C++14. Do not duplicate compiler flags (`#45 <https://github.com/ros-drivers/transport_drivers/issues/45>`_)
* Deduplicate ASIO CMake module (`#43 <https://github.com/ros-drivers/transport_drivers/issues/43>`_)
  * Added ASIO CMake module
  * Use asio_cmake_module
* Use RCLCPP logging macros (`#42 <https://github.com/ros-drivers/transport_drivers/issues/42>`_)
* Fix copyright years (`#41 <https://github.com/ros-drivers/transport_drivers/issues/41>`_)
* Removed Boost (`#39 <https://github.com/ros-drivers/transport_drivers/issues/39>`_)
  * Removed Boost
  * Do not depend on Boost at all
  * Added cmath header
  * Fix linting warning
  * Added ASIO_STANDALONE to more units
  * Call stop on io_service
  * Fix dependency
  * Export asio as a downstream dependency
  * Remove more boost references
  * Force non-Boost version of ASIO
  * Force non-Boost version of ASIO
  * Ignore result to avoid compiler warning
* Create Full UDP Nodes (`#38 <https://github.com/ros-drivers/transport_drivers/issues/38>`_)
  * Reorganize namespaces
  * Apply reviewer feedback
  * Create UdpReceiverNode as LifecycleNode
  * Create UdpSenderNode as Lifecycle Node
  * Adding bridge node
  * Add comment to bridge node about purpose
* Reorg namespaces (`#37 <https://github.com/ros-drivers/transport_drivers/issues/37>`_)
  * Move UDP driver node to correct location
  * Reorganize namespaces
  * Remove unused include_directories
  * Apply reviewer feedback
* move io_context to shared lib (`#36 <https://github.com/ros-drivers/transport_drivers/issues/36>`_)
* Contributors: Esteve Fernandez, Evan Flynn, Haoru Xue, Joshua Whitley

0.0.6 (2020-08-27)
------------------

0.0.5 (2020-07-16)
------------------

0.0.4 (2019-12-12)
------------------

0.0.3 (2019-08-21)
------------------

0.0.2 (2019-08-19)
------------------
