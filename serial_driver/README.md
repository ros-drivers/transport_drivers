# serial_driver

Serial port (UART) access for ROS 2 on standalone ASIO via `io_context`. The package provides:

- a ROS-independent `serial_driver` library, used directly by `crsf_receiver`;
- a lifecycle bridge node that exposes a port as byte-array topics.

## Node `serial_bridge` (`drivers::serial_driver::SerialBridgeNode`)

| Direction | Topic | Type |
|-----------|-------|------|
| pub | `serial_read` | `std_msgs/UInt8MultiArray` (bytes received, depth 100) |
| sub | `serial_write` | `std_msgs/UInt8MultiArray` (bytes to send, best effort, depth 32) |

| Parameter | Values |
|-----------|--------|
| `device_name` | e.g. `/dev/ttyACM0` |
| `baud_rate` | e.g. `115200` |
| `flow_control` | `none`, `software`, `hardware` |
| `parity` | `none`, `odd`, `even` |
| `stop_bits` | `1`, `1.5`, `2` |

The port opens on `configure`; an invalid parameter or a failure to open the port fails the
transition.

```bash
ros2 launch serial_driver serial_driver_bridge_node.launch.py \
  params_file:=$(ros2 pkg prefix serial_driver)/share/serial_driver/params/example.params.yaml
```

The launch file starts `serial_bridge_node` and configures and activates it automatically.
`params/example.params.yaml` is the example parameter file.

## Library

`drivers::serial_driver::SerialDriver` (`serial_driver/serial_driver.hpp`) takes an `IoContext`.
`init_port(device_name, SerialPortConfig)` creates a `SerialPort`
(`serial_driver/serial_port.hpp`), which provides `open`/`close`, `send`/`async_send` and
`receive`/`async_receive`.

```cpp
drivers::common::IoContext ctx{2};
drivers::serial_driver::SerialDriver driver{ctx};
driver.init_port("/dev/ttyUSB0", {115200, FlowControl::NONE, Parity::NONE, StopBits::ONE});
driver.port()->open();
```

## Tests

`test/test_serial_driver.cpp`, `test/test_serial_port.cpp`, run with
`colcon test --packages-select serial_driver`.
