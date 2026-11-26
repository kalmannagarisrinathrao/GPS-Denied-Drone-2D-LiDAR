# GPS-Denied Drone, 2D LiDAR

ROS 2 package `lidar_processor`: LiDAR obstacle detection, a go-to-goal node
with obstacle avoidance (PX4 Offboard), RViz visualisation, and a Gazebo maze
world. Simulation only.

Tested on Ubuntu 24.04, ROS 2 Jazzy, Gazebo Harmonic, PX4 SITL.

## 1. Install once

```bash
sudo apt install ros-jazzy-ros-gz-bridge ros-jazzy-tf2-ros ros-jazzy-rviz2 python3-colcon-common-extensions
```

Micro XRCE-DDS Agent (`MicroXRCEAgent` must be on your PATH):
https://docs.px4.io/main/en/middleware/uxrce_dds.html

PX4 (any folder you like; examples below use `$PX4_DIR`):

```bash
export PX4_DIR=$HOME/PX4-Autopilot          # change to your path
git clone --recursive https://github.com/PX4/PX4-Autopilot.git $PX4_DIR
cd $PX4_DIR && git checkout c726623d         # the version this was tested on
bash ./Tools/setup/ubuntu.sh                 # asks for sudo; log out and in after
```

The topic names in `goto_node.py` (`vehicle_status_v5`, `vehicle_command_v1`,
`vehicle_local_position_v1`) follow the PX4 message versions. A very different
PX4 version needs `px4_msgs` and those names updated together.

## 2. Get and build this package

```bash
cd ~/Projects                                 # any folder you like
git clone --recurse-submodules https://github.com/kalmannagarisrinath-jpg/GPS-Denied-Drone-2D-LiDAR.git
cd GPS-Denied-Drone-2D-LiDAR
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install                # first build takes ~5 min (px4_msgs)
```

If you cloned without `--recurse-submodules`: `git submodule update --init`.
Do not run any of this with a Python virtualenv active.

## 3. Install the maze world into your PX4 folder

```bash
cp src/lidar_processor/worlds/maze.sdf $PX4_DIR/Tools/simulation/gz/worlds/
```

Other mazes: `cd src/lidar_processor/worlds && python3 make_maze.py --seed 7 --loops 6`
then copy the new `maze.sdf` the same way. The generator prints the goal.

## 4. Run

Terminal 1:
```bash
MicroXRCEAgent udp4 -p 8888
```

Terminal 2 (use `walls` instead of `maze` for the original world):
```bash
cd $PX4_DIR
PX4_GZ_WORLD=maze make px4_sitl gz_x500_lidar_2d
```

Open QGroundControl once and leave it running. PX4 refuses to arm with
"No connection to the GCS" otherwise.

Terminal 3:
```bash
cd ~/Projects/GPS-Denied-Drone-2D-LiDAR
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch lidar_processor drone_system.launch.py world:=maze
```

Send a goal by clicking **2D Goal Pose** in RViz (map frame, x = East, y = North,
origin = spawn point), or by topic (x = North, y = East, z = height, metres):

```bash
ros2 topic pub --once /goto/goal geometry_msgs/msg/Point "{x: -16.0, y: 16.0, z: 2.0}"
ros2 topic pub --once /goto/land std_msgs/msg/Empty "{}"
```

In the maze the drone starts in the north-west cell facing East. The
south-east cell is North -16, East 16.

## RViz colours

| Display | Meaning |
|---|---|
| grey points | obstacles seen so far |
| cyan points | LiDAR now |
| red points | closer than the 2 m stop distance |
| yellow ball | goal |
| green / orange arrow | going straight / detouring |

## Tests

```bash
cd src/lidar_processor
python3 -m unittest test_goto_logic test_path_planner
```

`test_avoidance_logic.py` is out of date (it calls a removed function).

## Limits

- The avoidance is reactive and has no map. It can be trapped by dead ends.
- Sensing is a flat 2D scan; the rear arc is blind.
- Position still comes from simulated GPS. GPS-denied localisation is not done.
- Simulation only.
