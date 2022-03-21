^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package serial_driver
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.2.0 (2022-03-20)
------------------
* Disable broken test.
* Fix linter errors.
* Contributors: WhitleySoftwareServices

1.1.0 (2022-03-20)
------------------
* Support serial_break (`#76 <https://github.com/ros-drivers/transport_drivers/issues/76>`_)
  * Support serial_break
  * Add protection to serial break and unit tests
* Fix the converter Converter between `std_msgs::msg::UInt8MultiArray` and `std::vector<uint8_t>` (`#73 <https://github.com/ros-drivers/transport_drivers/issues/73>`_)
  * example_interfaces is redundant as std_msgs includes UInt8MultiArray
  * udp_msgs.hpp should not include "converters.hpp"
  * Fix the converter std_msgs::msg::UInt8MultiArray <-> std::vector<uint8_t>
* Add support for Foxy (`#68 <https://github.com/ros-drivers/transport_drivers/issues/68>`_)
  * Add support for Foxy
  * Use same API signature for all ROS distros
* Contributors: ChenJun, Esteve Fernandez, RFRIEDM-Trimble

1.0.1 (2021-08-30)
------------------
* Remove deprecated api from galactic (`#57 <https://github.com/ros-drivers/transport_drivers/issues/57>`_)
* Serial driver debugs (`#56 <https://github.com/ros-drivers/transport_drivers/issues/56>`_)
  * debugged message conversion
  debugged serial bridge segfault (publisher accessed before initialized)
  * refactored message conversion
* update READMEs for each package (`#54 <https://github.com/ros-drivers/transport_drivers/issues/54>`_)
  * update READMEs for each package
  * add more to readme, renamed config to params
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
* Export ASIO definitions (`#44 <https://github.com/ros-drivers/transport_drivers/issues/44>`_)
* Enforce C++14. Do not duplicate compiler flags (`#45 <https://github.com/ros-drivers/transport_drivers/issues/45>`_)
* Deduplicate ASIO CMake module (`#43 <https://github.com/ros-drivers/transport_drivers/issues/43>`_)
  * Added ASIO CMake module
  * Use asio_cmake_module
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
* remove autoware namespace (`#35 <https://github.com/ros-drivers/transport_drivers/issues/35>`_)
  * remove autoware namespace
  * move msgs namespace to utils
* Make Nodes Component-Compatible (`#22 <https://github.com/ros-drivers/transport_drivers/issues/22>`_)
  * Making constructors for SerialDriverNode component-compatible.
  * Making constructors for UdpDriverNode component-compatible.
* Contributors: Daisuke Nishimatsu, Esteve Fernandez, Evan Flynn, Haoru Xue, Joshua Whitley

0.0.6 (2020-08-27)
------------------
* Uncrustify fixes.
* Remove lifecycle references (`#19 <https://github.com/ros-drivers/transport_drivers/issues/19>`_)
* Fixing boost dependency. (`#18 <https://github.com/ros-drivers/transport_drivers/issues/18>`_)
* Contributors: Esteve Fernandez, Joshua Whitley

0.0.5 (2020-07-16)
------------------
* Remove Autoware.AUTO Dependencies (`#15 <https://github.com/ros-drivers/transport_drivers/issues/15>`_)
  * Removing autoware dependencies.
  * Fixing linting errors.
  * Addressing review feedback.
* Be specific about which parts of Boost are necessary (`#10 <https://github.com/ros-drivers/transport_drivers/issues/10>`_)
  * serial: be specific about Boost dependency.
  * udp: be specific about Boost dependency.
* Contributors: G.A. vd. Hoorn, Joshua Whitley

0.0.4 (2019-12-12)
------------------
* Making serial_driver version number consistent with repo.
* Initial commit of serial_driver.
* Contributors: Joshua Whitley

0.0.3 (2019-08-21)
------------------

0.0.2 (2019-08-19)
------------------
