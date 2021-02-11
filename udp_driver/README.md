# udp_driver

### udp_component
Provided within this package is a default ROS2 component.

At runtime the provided `udp_component` requires two parameters to be set: `ip` and `port`.

These parameters can be set in two ways: either by [loading in from a parameter file](https://index.ros.org/doc/ros2/Tutorials/Node-arguments/#setting-parameters-from-yaml-files) or by [passing them as arguements](https://index.ros.org/doc/ros2/Tutorials/Node-arguments/#parameters)

Provided with this package is a default setup for the parameters file, located in the `udp_driver/config` directory.

Go ahead and modify the provided parameter file to your desired UDP settings and run the component:
```
ros2 run udp_driver udp_node --
```

#### Renaming the provided udp_component

If you'd like to rename the `udp_component` provided, this can be done at runtime via the command line:

```
ros2 run udp_driver udp_node --ros-args --remap __node:=<new node name>
```

**NOTE:** you'll need to still provide ROS parameters for it to launch and if you are using the provided parameters file you'll need to change the first line of the file to whatever your new node name will be.

