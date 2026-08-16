# turtlebot3_custom_worlds

Custom Gazebo Classic worlds for TurtleBot3 in ROS 2 Humble.

This package provides a set of progressive simulation stages for mobile robot navigation, fuzzy logic experiments, and reinforcement learning with TurtleBot3.

---

## Features

- Custom Gazebo worlds organized by stages.
- Short launch argument: `stage:=N`.
- TurtleBot3 Burger spawned from the official SDF model.
- 10 m x 10 m metric grid.
- Positive axes visualization:
  - `+X`: red.
  - `+Y`: green.
- Movable green goal marker for RL episodes.
- Episode reset manager executable with `ros2 run`.
- Worlds designed for navigation, obstacle avoidance and RL curriculum learning.

---

## Package Structure

```text
turtlebot3_custom_worlds/
├── CMakeLists.txt
├── package.xml
├── README.md
├── launch/
│   └── d3im3r_world.launch.py
├── worlds/
│   ├── d3im3r_stage_00_empty.world
│   ├── d3im3r_stage_01_direct_goal.world
│   ├── d3im3r_stage_02_front_obstacle.world
│   ├── d3im3r_stage_03_left_right_choice.world
│   ├── d3im3r_stage_04_corridor.world
│   ├── d3im3r_stage_05_narrow_door.world
│   ├── d3im3r_stage_06_random_obstacles.world
│   └── d3im3r_stage_07_simple_maze.world
├── models/
│   ├── d3im3r_goal_marker/
│   └── d3im3r_grid/
├── scripts/
│   └── rl_episode_manager.py
└── config/
    └── d3im3r_stage_02_front_obstacle.yaml
```

---

## World Convention

```text
+X  -> Forward direction
+Y  -> Left direction
+Z  -> Up
yaw = 0 -> Robot facing +X
```

The world grid is:

```text
10 m x 10 m
1 square = 1 meter
```

Default robot pose:

```text
x = -1.5
y =  0.0
z =  0.01
yaw = 0.0
```

Default goal pose:

```text
x = 1.5
y = 0.0
z = 0.03
```

---

## Dependencies

```bash
sudo apt update
sudo apt install ros-humble-gazebo-ros-pkgs
sudo apt install ros-humble-turtlebot3
sudo apt install ros-humble-turtlebot3-gazebo
sudo apt install ros-humble-gazebo-msgs
sudo apt install python3-colcon-common-extensions
```

---

## Build

From the workspace root:

```bash
cd ~/ros2_ws
colcon build --packages-select turtlebot3_custom_worlds
source install/setup.bash
```

Clean build:

```bash
cd ~/ros2_ws
rm -rf build/turtlebot3_custom_worlds install/turtlebot3_custom_worlds
colcon build --packages-select turtlebot3_custom_worlds
source install/setup.bash
```

---

## Launch Worlds

Close previous Gazebo processes before launching a new stage:

```bash
pkill -f gzserver
pkill -f gzclient
```

Launch a stage:

```bash
ros2 launch turtlebot3_custom_worlds d3im3r_world.launch.py stage:=1
```

Available stages:

| Stage | World | Purpose |
|---|---|---|
| `0` | Empty world | Basic simulation test |
| `1` | Direct goal | Goal reaching without obstacles |
| `2` | Front obstacle | Basic obstacle avoidance |
| `3` | Left/right choice | Decision-making between obstacles |
| `4` | Corridor | Corridor navigation |
| `5` | Chicane | Controlled deviation path |
| `6` | Random obstacles | Distributed obstacle navigation |
| `7` | Simple maze | Advanced navigation test |

Examples:

```bash
ros2 launch turtlebot3_custom_worlds d3im3r_world.launch.py stage:=0
ros2 launch turtlebot3_custom_worlds d3im3r_world.launch.py stage:=5
ros2 launch turtlebot3_custom_worlds d3im3r_world.launch.py stage:=7
```

Custom robot pose:

```bash
ros2 launch turtlebot3_custom_worlds d3im3r_world.launch.py \
stage:=5 \
x_pose:=-1.5 \
y_pose:=0.0 \
z_pose:=0.01 \
yaw:=0.0
```

---

## Test Robot Motion

In another terminal:

```bash
cd ~/ros2_ws
source install/setup.bash
export TURTLEBOT3_MODEL=burger
```

Move forward:

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
"{linear: {x: 0.15, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

Turn:

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
"{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.5}}"
```

Stop:

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
"{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

---

## Useful Topics

```bash
ros2 topic list
```

Expected topics:

```text
/cmd_vel
/odom
/scan
/tf
/tf_static
/joint_states
/clock
```

Check odometry:

```bash
ros2 topic echo /odom --once
```

Check LiDAR:

```bash
ros2 topic echo /scan --once
```

---

## RL Episode Manager

The script `scripts/rl_episode_manager.py` is installed as a ROS 2 executable.

Check executable:

```bash
ros2 pkg executables turtlebot3_custom_worlds
```

Expected output:

```text
turtlebot3_custom_worlds rl_episode_manager
```

Run a stage:

```bash
ros2 launch turtlebot3_custom_worlds d3im3r_world.launch.py stage:=5
```

In another terminal:

```bash
cd ~/ros2_ws
source install/setup.bash
ros2 run turtlebot3_custom_worlds rl_episode_manager
```

Expected behavior:

- The robot returns to the initial pose.
- The goal marker changes position.
- Gazebo remains open.
- The current stage obstacles remain fixed.

---

## Moving the Goal Manually

With Gazebo running:

```bash
ros2 service call /gazebo/set_entity_state gazebo_msgs/srv/SetEntityState "{
state: {
name: 'goal_marker',
pose: {
position: {x: 1.5, y: 1.0, z: 0.03},
orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
},
twist: {
linear: {x: 0.0, y: 0.0, z: 0.0},
angular: {x: 0.0, y: 0.0, z: 0.0}
},
reference_frame: 'world'
}
}"
```

The goal marker must be defined with:

```xml
<static>false</static>
```

---

## Recommended RL Curriculum

```text
Stage 1 -> Direct goal
Stage 2 -> Front obstacle
Stage 3 -> Left/right decision
Stage 4 -> Corridor
Stage 5 -> Chicane
Stage 6 -> Distributed obstacles
Stage 7 -> Simple maze
```

Recommended RL state:

```text
[d_front_norm, d_left_norm, d_right_norm, theta_goal_norm, d_goal_norm]
```

Recommended discrete actions:

```text
0 -> forward
1 -> turn left
2 -> turn right
```

---

## Troubleshooting

### Gazebo keeps loading the wrong world

```bash
pkill -f gzserver
pkill -f gzclient
```

Then rebuild and source:

```bash
cd ~/ros2_ws
colcon build --packages-select turtlebot3_custom_worlds
source install/setup.bash
```

---

### `ros2 run` does not find `rl_episode_manager`

Check permissions:

```bash
ls -l ~/ros2_ws/src/turtlebot3_custom_worlds/scripts/rl_episode_manager.py
```

Fix permissions:

```bash
chmod +x ~/ros2_ws/src/turtlebot3_custom_worlds/scripts/rl_episode_manager.py
```

Rebuild:

```bash
cd ~/ros2_ws
rm -rf build/turtlebot3_custom_worlds install/turtlebot3_custom_worlds
colcon build --packages-select turtlebot3_custom_worlds
source install/setup.bash
```

---

### The goal does not move

Make sure the goal model uses:

```xml
<static>false</static>
<gravity>false</gravity>
<kinematic>true</kinematic>
```

Also verify:

```bash
ros2 service list | grep entity
```

Expected:

```text
/gazebo/set_entity_state
```

---

### Robot appears gray or without textures

Clean Gazebo environment variables before launching:

```bash
unset GAZEBO_MODEL_PATH
unset GAZEBO_RESOURCE_PATH
unset GAZEBO_PLUGIN_PATH
```

Then launch again.

---

### Many `Missing model.config` errors

This usually means `GAZEBO_MODEL_PATH` points to folders that are not Gazebo models.

Make sure `models/` only contains valid model folders such as:

```text
models/
├── d3im3r_goal_marker/
└── d3im3r_grid/
```

---

## Author

Developed for TurtleBot3 navigation and reinforcement learning experiments in ROS 2 Humble.

Author: Deymer Miranda
