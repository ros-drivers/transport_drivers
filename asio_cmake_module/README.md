# asio_cmake_module

CMake find module for the standalone [ASIO](https://think-async.com/Asio/) networking library,
vendored from [ros-drivers/transport_drivers](https://github.com/ros-drivers/transport_drivers)
(version 1.2.0). It builds nothing. It installs `cmake/Modules/FindASIO.cmake` and adds that
directory to `CMAKE_MODULE_PATH` for packages that find it.

## Usage

```xml
<!-- package.xml -->
<buildtool_depend>asio_cmake_module</buildtool_depend>
<depend>asio</depend>
```

```cmake
find_package(asio_cmake_module REQUIRED)
find_package(ASIO REQUIRED)

target_include_directories(my_target SYSTEM PUBLIC ${ASIO_INCLUDE_DIRS})
target_compile_definitions(my_target PUBLIC ${ASIO_DEFINITIONS})
```

`FindASIO` sets:

| Variable | Value |
|----------|-------|
| `ASIO_FOUND` | whether `asio.hpp` was found |
| `ASIO_INCLUDE_DIRS` | directory containing `asio.hpp` |
| `ASIO_DEFINITIONS` | `ASIO_STANDALONE` (use ASIO without Boost) |

The system `asio` headers must be installed (`libasio-dev`, resolved by rosdep from `asio`).

Used by `io_context`, `udp_driver` and `serial_driver` in this repository.
