# UR5e Isaac Sim Demo

ROS 2 workspace that launches a Universal Robots UR5e arm into NVIDIA Isaac Sim, mounts a wrist camera, brings up MoveIt against the simulator's ROS 2 bridge, and provides automated dataset collection with 3D projection verification.

Compatible with **ROS 2 Humble (Ubuntu 22.04)** and **ROS 2 Jazzy (Ubuntu 24.04)** with Isaac Sim 5.1 / 6.0.

---

## Features

- **UR5e Arm in Isaac Sim**: Spawns using official UR description with topic-based `ros2_control` and MoveIt integration.
- **Wrist Camera**: Mounted to `tool0` with `camera_link` and `camera_optical_frame`. Publishes `/wrist_cam/image_raw` and `/wrist_cam/camera_info` at 640x480 resolution and 15 Hz, synchronized with simulation time (`/clock`).
- **Interactive Target Object**: Spawns a collidable `DynamicCuboid` in front of the robot base.
- **5-Pose Dataset Collection**: Automated script that commands the robot through 5 distinct viewpoints, confirms the arm has settled, and records images, joint angles, camera poses (in `poses.jsonl`), and camera calibration (in `camera_info.yaml`).
- **3D Projection Verification**: Mathematical check projecting the 3D cube coordinates into each 2D camera image to verify TF and calibration alignment.

---

## Quick Start (Ubuntu 22.04 + ROS 2 Humble)

To run the complete stack (Isaac Sim GUI + MoveIt + RViz):

```bash
./run_demo.sh
```

Or run headless without RViz (for CI or automated pipelines):

```bash
./run_demo.sh headless:=true launch_rviz:=false
```

> [!NOTE]
> If you have Conda/Miniconda active in your default shell (e.g. Python 3.12+), `./run_demo.sh` automatically sanitizes `PATH` to avoid binary and Python version conflicts with ROS 2 Humble (Python 3.10) and Isaac Sim (Python 3.11).

---

## Build

From the workspace root:

```bash
cd ros2_ws
source /opt/ros/humble/setup.bash   # or /opt/ros/jazzy/setup.bash
colcon build --symlink-install --cmake-args -DBUILD_TESTING=OFF
source install/setup.bash
cd ..
```

---

## ROS 2 Topics

| Topic | Type | Rate | Description |
|---|---|---|---|
| `/wrist_cam/image_raw` | `sensor_msgs/msg/Image` | 15 Hz | RGB camera feed (640x480, `camera_optical_frame`, sim time) |
| `/wrist_cam/camera_info` | `sensor_msgs/msg/CameraInfo` | 15 Hz | Camera intrinsic calibration matrix and distortion coefficients |
| `/isaac_joint_states` | `sensor_msgs/msg/JointState` | 60 Hz | Joint positions, velocities, and efforts from Isaac Sim |
| `/isaac_joint_commands` | `sensor_msgs/msg/JointState` | - | Joint trajectory commands sent to Isaac Sim |
| `/clock` | `rosgraph_msgs/msg/Clock` | 60 Hz | Simulation clock for `use_sim_time:=true` |


## Dataset Collection & Verification

### 1. Collect 5 Poses Dataset
While `./run_demo.sh` is running, open a new terminal:

```bash
source /opt/ros/humble/setup.bash
python3 ros2_ws/src/ur5e_isaac_bridge/scripts/capture_data.py
```

This commands the UR5e through 5 pre-configured viewpoints looking at the cube from different angles. At each pose, it waits for the robot to stop and saves:
- `dataset/camera_info.yaml` — Camera calibration parameters
- `dataset/image_0.png` ... `image_4.png` — RGB images from 5 perspectives
- `dataset/poses.jsonl` — Joint angles and camera pose (`base_link` $\rightarrow$ `camera_optical_frame`) for each image

## Launch Arguments

| Argument | Default | Meaning |
|---|---|---|
| `ur_type` | `ur5e` | Robot model passed to the Universal Robots description. |
| `launch_isaac` | `true` | Start Isaac Sim from this launch. Set `false` to attach MoveIt to a running simulator. |
| `launch_rviz` | `true` | Start RViz with the MoveIt configuration. |
| `use_sim_time` | `true` | Synchronize ROS 2 nodes with Isaac Sim `/clock`. |
| `headless` | `false` | Run Isaac Sim without GUI window. |
| `isaac_python` | `~/isaacsim/python.sh` | Path to Isaac Sim Python launcher. |
| `joint_commands_topic` | `/isaac_joint_commands` | Command topic between ros2_control and Isaac Sim. |
| `joint_states_topic` | `/isaac_joint_states` | State topic from Isaac Sim into ros2_control. |
