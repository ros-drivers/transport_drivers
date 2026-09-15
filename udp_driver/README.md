# udp_driver

Sends and receives UDP datagrams as ROS 2 `udp_msgs/UdpPacket` messages. Built on standalone
ASIO via `io_context`. The package also provides a ROS-independent `udp_driver` library.

On the rover it carries the BMS telemetry (`rover_battery`) and the LED frames (`rover_led`).

## Nodes

All nodes are lifecycle nodes. Configure and activate them, e.g. with launch_ros
`LifecycleNode(..., autostart=True)`.

| Executable | Component | Topics | Parameters |
|------------|-----------|--------|------------|
| `udp_receiver_node_exe` | `drivers::udp_driver::UdpReceiverNode` | pub `udp_read` (`udp_msgs/UdpPacket`, depth 100) | `ip` (local address to bind), `port` |
| `udp_sender_node_exe` | `drivers::udp_driver::UdpSenderNode` | sub `udp_write` (`udp_msgs/UdpPacket`, best effort, depth 32) | `ip` (destination address), `port` |
| `udp_bridge_node_exe` | receiver + sender in one process sharing one `IoContext` | both of the above | both of the above |

On `configure` the receiver binds the socket and starts receiving asynchronously; the sender
opens its socket. A socket error fails the transition. `params/udp_params.yml` is an example
parameter file.

Rover usage:

| Node | Launched by | Topic (remapped) | Endpoint |
|------|-------------|------------------|----------|
| `rover_udp_battery_receiver_node` | `rover_battery` | `udp_read` → `rover_battery_udp_data` | BMS packets |
| `rover_udp_led_channel_1_sender_node` / `_2_` | `rover_led` | `udp_write` → `udp_write/led_channel_<n>` | `192.168.77.201:3333` |

```bash
ros2 run udp_driver udp_sender_node_exe --ros-args -p ip:=192.168.77.201 -p port:=3333
ros2 lifecycle set /udp_sender_node configure
ros2 lifecycle set /udp_sender_node activate
```

## Library

`drivers::udp_driver::UdpDriver` (`udp_driver/udp_driver.hpp`) takes an `IoContext`. Its
`init_sender(ip, port)` / `init_receiver(ip, port)` create `UdpSocket`s
(`udp_driver/udp_socket.hpp`), which provide `open`, `bind`, `send`/`asyncSend` and
`receive`/`asyncReceive`. The design is described in `design/udp_driver-design.md`.

## Tests

`test/`: socket, driver, packet data and node tests, run with
`colcon test --packages-select udp_driver`.
