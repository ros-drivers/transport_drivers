* **UDP Driver**

A package which encapsulates basic receving and sending of udp data.

Provided within this package are the following executabes:
- udp_receiver_node_exe: can receive UDP data
- udp_sender_node_exe: can send UDP data asynchronosouly
- udp_bridge_node_exe: combined both receiver and sender nodes into one

Provided within this package also is a `udp_driver` library without the ROS2 dependencies which could be used elsewhere.