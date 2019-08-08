udp_driver
===============

This is the design document for the `udp_driver` package.


# Purpose / Use cases
<!-- Required -->
<!-- Things to consider:
    - Why did we implement this feature? -->

There are a number of methods by which hardware devices can communicate to software.
Of these, the most common connection method that is still real-time capable is via UDP.
In light of this, the `UdpDriver` class seeks to be some common core functionality for
a driver which passively listens to UDP packets arriving from a sensor device, such as
a LiDAR.

# Design
<!-- Required -->
<!-- Things to consider:
    - How does it work? -->

This class broadly does one thing: receive UDP packets, convert them to the relevant type,
publish the result(s) when necessary, and handles basic errors.

Because a steady stream of packets is assumed rather than bursty behavior, if a packet
cannot be handled in the time before another packet arrives, there is no hope of having
a functional driver (since the buffer will become overfull).

As a result, we get a small simplification/optimization of only needing one thread/process for
our driver.

Finally, the driver is built on top of a managed node. A managed node is
used as the base to allow for a standard interface to handle initialization, and error handling.


## Assumptions / Known limits
<!-- Required -->

It is assumed that the source of UDP packets is steady, rather than bursty.
If the packet source is bursty, depending on the system settings, the UDP buffer may
become overfull and start dropping packets, meaning that you lose valuable information.
This can be to some extent mitigated by allowing large buffer sizes.

It is also assumed that the output type of the driver is a ROS 2 message.

# Inputs / Outputs / API
<!-- Required -->
<!-- Things to consider:
    - How do you use the package / API? -->
    
    
Inputs:

- Node parameters (Node and topic names)
- Connection parameters (IP and Port)


In general, instantiating a driver requires defining six things:

1. A packet struct with correct size
2. An output type that is a publishable ros2 message
3. Define the `convert()` method
4. Define the `get_output_remainder()` method
5. Define the `init_output()` method



# Error detection and handling
<!-- Required -->

Errors in the `get_packet()` function are detected and caught in the main loop. These errors 
indicate a malfunctioning in udp communication due to a failure in receiving a packet or writing
 it into a buffer.

# Security considerations
<!-- Required -->
<!-- Things to consider:
- Spoofing (How do you check for and handle fake input?)
- Tampering (How do you check for and handle tampered input?)
- Repudiation (How are you affected by the actions of external actors?).
- Information Disclosure (Can data leak?).
- Denial of Service (How do you handle spamming?).
- Elevation of Privilege (Do you need to change permission levels during execution?) -->

This driver is vulnerable to Denial of Service attacks.

If the port is spammed with packets from the wrong IP, then the driver's thread can practically
be spinlocked as it handles packets from the wrong IP. This can also result in the receipt of a
packet from the target IP to time out.

Handling this kind of attack will likely require an OS or system-level safeguard.


# References / External links
<!-- Optional -->


# Future extensions / Unimplemented parts
<!-- Optional -->

The parent class of this Node will eventually become a `ThreadedNode`, when implemented.


# Related issues
<!-- Required -->
- #4 - Implement velodyne driver