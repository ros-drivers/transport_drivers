# udp_driver

### udp_component
Provided within this package is a default ROS2 component.

To use, first start a ROS2 component container:
```
ros2 run rclcpp_components component_container
```

In another terminal, go ahead and load the component:
```
ros2 component load /ComponentManager udp_driver udp_driver::UdpComponent
```
