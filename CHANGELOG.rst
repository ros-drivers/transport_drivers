^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package udp_driver
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.1.0 (2021-02-18)
------------------
* Tracking PR: https://github.com/ros-drivers/transport_drivers/pull/31
* Implemented IoContext (The main I/O Context that can be configured as single or multi thread with async task execution support)
* Implemented UdpSocket (The UDP socket functionality that working in blocking and non-blocking (Async with task execution) modes)
* Implemented UdpDriver Facade (Facade design pattern implementation that acts as a sender and receiver UDP socket pair container)
* Implemented ROS2 and raw buffer message converters for Int, UInt and Float variants in std_msgs namespace.
* Implemented UdpDriverNode example and its test.
* Implemented IoContext, UdpSocket and UdpDriver tests.
* Contributors: Reza Ebrahimi

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
