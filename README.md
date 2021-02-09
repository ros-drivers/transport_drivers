# transport_drivers
A set of ROS2 drivers for transport-layer protocols. Currently utilizes Boost::ASIO for most transport-layer functionality.

## udp_driver
A package which contains a templated C++ class (which inherits from `rclcpp::Node`) which encapsulates basic sending and receiving of UDP network data.

## serial_driver
A package which contains a templated C++ class (which inherits from `rclcpp::Node`) which encapsulates basic receiving (and soon sending) of serial data.
