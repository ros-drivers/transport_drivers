# io_context

Thin wrapper around a standalone ASIO `io_service` and its worker thread pool, shared by
`udp_driver` and `serial_driver`. It is a library only, with no nodes.

## API (`io_context/io_context.hpp`, namespace `drivers::common`)

| Member | Description |
|--------|-------------|
| `IoContext()` / `IoContext(size_t threads_count)` | Starts the `io_service` with `threads_count` worker threads (default: hardware concurrency). Not copyable. |
| `asio::io_service & ios() const` | The underlying service, used to construct sockets and serial ports. |
| `void post(F f)` | Runs `f` on a worker thread. |
| `bool isServiceStopped()` / `uint32_t serviceThreadCount()` | State queries. |
| `void waitForExit()` | Stops the service and joins the worker threads. |

```cpp
#include "io_context/io_context.hpp"

drivers::common::IoContext ctx{2};
drivers::udp_driver::UdpDriver driver{ctx};
```

```cmake
find_package(io_context REQUIRED)   # also needs asio_cmake_module + ASIO
```

## Tests

`test/test_io_context.cpp`: `colcon test --packages-select io_context`.
