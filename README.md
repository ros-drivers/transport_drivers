# transport_drivers

A set of ROS2 drivers for transport-layer protocols. Currently utilizes Boost::ASIO for most transport-layer functionality.

## Supported Drivers:

* **UDP Driver**
    * Tracking PR: https://github.com/ros-drivers/transport_drivers/pull/31
    * Implemented `IoContext` (The main I/O Context that can be configured as single or multi thread with async task execution support)
    * Implemented `UdpSocket` (The UDP socket functionality that working in blocking and non-blocking (Async with task execution) modes)
    * Implemented `UdpDriver` Facade (Facade design pattern implementation that acts as a sender and receiver UDP socket pair container)
    * Implemented `ROS2` and `raw` buffer message converters for Int, UInt and Float variants in std_msgs namespace.
    * Implemented `UdpDriverNode` example and its test.
    * Implemented IoContext, UdpSocket and UdpDriver tests.
    
* **Serial Driver**
A package which contains a templated C++ class (which inherits from rclcpp::Node) which encapsulates basic receiving (and soon sending) of serial data.