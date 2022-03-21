^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package udp_driver
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.2.0 (2022-03-20)
------------------
* Fix linter errors.
* Contributors: WhitleySoftwareServices

1.1.0 (2022-03-20)
------------------
* Add missing header.
* Add new constructors and members to bind host endpoint (`#65 <https://github.com/ros-drivers/transport_drivers/issues/65>`_)
  * Add new constructors and members to bind host endpoint
  * address review: fix constructor and endpoint
* Add support for Foxy (`#68 <https://github.com/ros-drivers/transport_drivers/issues/68>`_)
  * Add support for Foxy
  * Use same API signature for all ROS distros
* Fix nullptr access in udp receiver node (`#67 <https://github.com/ros-drivers/transport_drivers/issues/67>`_)
* Add reuse address function (`#64 <https://github.com/ros-drivers/transport_drivers/issues/64>`_)
  * Add reuse address function
  * Address review: Enable reuse address in open function
* Contributors: Daisuke Nishimatsu, Esteve Fernandez, WhitleySoftwareServices

1.0.1 (2021-08-30)
------------------
* Remove deprecated api from galactic (`#57 <https://github.com/ros-drivers/transport_drivers/issues/57>`_)
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
* Removing duplicate file
* Export ASIO definitions (`#44 <https://github.com/ros-drivers/transport_drivers/issues/44>`_)
* Enforce C++14. Do not duplicate compiler flags (`#45 <https://github.com/ros-drivers/transport_drivers/issues/45>`_)
* Deduplicate ASIO CMake module (`#43 <https://github.com/ros-drivers/transport_drivers/issues/43>`_)
  * Added ASIO CMake module
  * Use asio_cmake_module
* Use RCLCPP logging macros (`#42 <https://github.com/ros-drivers/transport_drivers/issues/42>`_)
* Fix copyright years (`#41 <https://github.com/ros-drivers/transport_drivers/issues/41>`_)
* Fix IoContext thread management
* Pass IoContext through driver instead of storing reference
* Replace ptrs to IoContext with refs
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
* Add warnings for invalid or missing parameters in udp_driver nodes
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
* Re-apply ament_cmake_auto to udp_driver
* remove autoware namespace (`#35 <https://github.com/ros-drivers/transport_drivers/issues/35>`_)
  * remove autoware namespace
  * move msgs namespace to utils
* Implement UdpSocket for both Syn. & Async modes (Send & Receive functionality) (`#31 <https://github.com/ros-drivers/transport_drivers/issues/31>`_)
  * Refactor & add UdpReceiver class
  * Add UdpSender class
  * Update node class to refactored driver
  * Update CMake
  * Update tests
  * Resolve a conflict on test/test_udp_driver.cpp
  * Update Readme
  * Rename header file to cpp format
  * Rename header file to cpp format
  * Delete old files
  * Update Udp Driver Node
  * Add message converters
  * Add IoContext class
  * Add UdpDriver class
  * Add UdpSocket class
  * Add IoContext test case
  * Add Udp data transmission test cases
  * Add Udp receive test case
  * Add Udp Sender test case
  * Add UdpDriver test cases
  * Update CMakeLists file
  * Add converters header & source files
  * Add IoContext header & source files
  * Add UdpSocket header & source files
  * Add UdpDriver header & source files
  * Add visibility header file
  * Add UdpDriverNode to examples directory
  * Add main google test program
  * Add IoContext test
  * Add UdpSocket test
  * Add UdpDriver test
  * Add UDP communication tests
  * Add UdpDriverNode test
  * Add changelog file
  * Add CMakeLists file
  * Add package.xml file
  * Delete old udp_driver package
  * Add design document
  * Add std_msgs/Int* variants converters
  * Update converters to ROS2 message in function names
  * Refactor converters to their ROS2 namespaces in converters directory
  * Add common converters header
  * Update licenses
  * Reformat code
  * Remove unnecessary print
  * Update google test header includes to library convention
  * Implement unsigned integer and floating point message variants converters
  * Fix comment in converters
  * Update test and example
  * Update changelog - Bump major version to 0.1.0
  * Update readme
  * Revert back changelog file
  * Update readme
  * Move udp_driver to separate package
  * Revert package name change
  * Apply linter fixes
  Co-authored-by: Joshua Whitley <josh.whitley@autoware.org>
* Make Nodes Component-Compatible (`#22 <https://github.com/ros-drivers/transport_drivers/issues/22>`_)
  * Making constructors for SerialDriverNode component-compatible.
  * Making constructors for UdpDriverNode component-compatible.
* Contributors: Daisuke Nishimatsu, Esteve Fernandez, Evan Flynn, Joshua Whitley, Reza Ebrahimi

0.0.6 (2020-08-27)
------------------
* Uncrustify fixes.
* Remove lifecycle references (`#19 <https://github.com/ros-drivers/transport_drivers/issues/19>`_)
* Fixing boost dependency. (`#18 <https://github.com/ros-drivers/transport_drivers/issues/18>`_)
* Contributors: Esteve Fernandez, Joshua Whitley

0.0.5 (2020-07-16)
------------------
* Be specific about which parts of Boost are necessary (`#10 <https://github.com/ros-drivers/transport_drivers/issues/10>`_)
  * serial: be specific about Boost dependency.
  * udp: be specific about Boost dependency.
* Fix doxygen
* Contributors: Esteve Fernandez, G.A. vd. Hoorn

0.0.4 (2019-12-12)
------------------
* Added UdpConfig class to encapsulate configuration options
* Removed workaround for ROS 2 Dashing PR2
* Contributors: Esteve Fernandez

0.0.3 (2019-08-21)
------------------
* Merge pull request `#2 <https://github.com/ros-drivers/transport_drivers/issues/2>`_ from esteve/fix-dependencies
  Added ament_lint_auto dependency
* Added ament_lint_auto dependency
* Contributors: Esteve Fernandez

0.0.2 (2019-08-19)
------------------
* Bump version
* Initial checkin
* Contributors: Esteve Fernandez
