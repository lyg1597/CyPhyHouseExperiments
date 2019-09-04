# Collecting Experiment Data

## Publish rate and message drop rate of ROS Topics

Set ROS parameter `enable_statistics` to true.
Then subscribe to `/statistics` topic or use `rqt_graph`.


## Runtime CPU and Memory usage of ROS nodes

`rqt_top` shows information provided by `top` but limited to ROS nodes.


## Real Time Factor

A ratio between simulated time versus wall clock reported by Gazebo simulator.

This requires enabling `use_sim_time` when launch Gazebo.
We enable this by passing the argument to ROS launch file `empty_world.launch`
of `gazebo_ros` package.


## Network Packet sent by CyPyHous3 middle-ware

Use `tshark` to capture UDP packet in a port range:
```
tshark -f "udp portrange 2000-2016"
```

This command requires higher permission to run the command.
In our server, we create a group `wireshark` for students to run this
experiment.

