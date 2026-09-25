# nova_exploration_rviz

`nova_exploration_rviz` is an optional RViz panel plugin for `nova_exploration`.

It does not add any new ROS interfaces to the explorer package. The panel only uses:

- the existing `nova_exploration/srv/ControlExploration` service
- existing `/rosout` log messages
- ROS graph observation for lightweight running/idle inference

## Features

- Start and stop exploration from RViz
- Delayed start or stop with the same `-t` semantics as the CLI
- Stop with `-q` from RViz
- Service discovery for one or more explorer instances
- Status badge colors for goal reached, return-to-start, preemption, waiting, and error flows

## Usage

Because this package lives inside the `nova_exploration` repository tree, it stays
out of the default `colcon build` scan for a typical ROS 2 workspace `src` tree. That keeps
the RViz plugin optional by default.

Build the core package first in the normal workspace flow, then build the RViz plugin
explicitly from its nested `plugin/` base path:

```bash
source /opt/ros/jazzy/setup.bash
cd ~/your_ros2_ws
colcon build --packages-select nova_exploration
colcon build \
  --base-paths src/nova_exploration/plugin/nova_exploration_rviz \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
source install/setup.bash
ros2 launch nova_exploration_rviz rviz_with_frontier_panel.launch.py
```

You can also open any RViz session and add the panel manually:

1. `Panels -> Add New Panel`
2. Select `nova_exploration_rviz/ExplorationControlPanel`

## Notes

- The panel stays optional. `nova_exploration` can still be built and used without launching RViz.
- A plain `colcon build` in your workspace root will not pick up this nested package unless you add its `plugin/nova_exploration_rviz` base path explicitly.
- Exact scheduled state tracking is guaranteed only for commands initiated from the panel itself.
- External delayed control requests are shown through observed runtime state and matching log events.
