# UR5e Isaac Sim demo

ROS 2 workspace that launches a UR5e into NVIDIA Isaac Sim and brings up MoveIt against the simulator's ROS 2 bridge. Compatible with both **ROS 2 Humble (Ubuntu 22.04)** and **ROS 2 Jazzy (Ubuntu 24.04)**, and Isaac Sim 5.1 / 6.0.

Isaac Sim is expected at `~/isaacsim`. The ROS side uses the official Universal Robots description and MoveIt config. Joint commands and joint states cross the bridge on `/isaac_joint_commands` and `/isaac_joint_states`, with simulation time on `/clock`.

## Quick Start (Native Ubuntu 22.04 + ROS 2 Humble)

To run the full stack (Isaac Sim GUI + MoveIt + RViz):

```bash
./run_demo.sh
```

Or run headless without RViz (for testing or CLI motion):

```bash
./run_demo.sh headless:=true launch_rviz:=false
```

> [!NOTE]
> If you have Conda/Miniconda active in your default shell (e.g. Python 3.12+), `./run_demo.sh` automatically strips Conda from your `PATH` and environment to avoid binary conflicts with ROS 2 Humble (Python 3.10) and Isaac Sim (Python 3.11).

---

## Build

### On Ubuntu 22.04 (ROS 2 Humble):

```bash
# Ensure Conda is deactivated if installed
export PATH=$(echo "$PATH" | tr ':' '\n' | grep -v 'miniconda' | paste -sd:)
unset CONDA_PREFIX
unset CONDA_DEFAULT_ENV

cd ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --cmake-args -DBUILD_TESTING=OFF
source install/setup.bash
```

### On Ubuntu 24.04 (ROS 2 Jazzy):

```bash
cd ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --cmake-args -DBUILD_TESTING=OFF
source install/setup.bash
```

`topic_based_ros2_control` configures tests that need ros-testing. The `-DBUILD_TESTING=OFF` argument skips those test-only dependencies.

## Launch

```bash
cd ros2_ws
source /opt/ros/humble/setup.bash  # or jazzy
source install/setup.bash
ros2 launch ur5e_isaac_bridge ur5e_isaac.launch.py
```

The launch file starts Isaac Sim with `~/isaacsim/python.sh`, imports the UR5e, and presses Play inside the standalone script. MoveIt and RViz start after the bridge topics appear. In RViz, plan with the `ur_manipulator` group.

Useful arguments:

| Argument | Default | Meaning |
|---|---|---|
| `ur_type` | `ur5e` | Robot model passed to the Universal Robots description. |
| `launch_isaac` | `true` | Start Isaac Sim from this launch. Set `false` to attach MoveIt to a simulator that is already running. |
| `launch_rviz` | `true` | Start RViz with the official MoveIt display config. |
| `use_sim_time` | `true` | Use `/clock` from Isaac Sim. |
| `headless` | `false` | Run the Isaac Sim app without a window. |
| `isaac_python` | `~/isaacsim/python.sh` | Isaac Sim Python launcher. |
| `joint_commands_topic` | `/isaac_joint_commands` | Commands from ros2_control into Isaac Sim. |
| `joint_states_topic` | `/isaac_joint_states` | Joint states from Isaac Sim into ros2_control. |

The first Isaac Sim startup can take longer than the launch's 180 second wait. If the log reports that the bridge timed out and MoveIt was not launched, leave the simulator running and start MoveIt against it:

```bash
ros2 launch ur5e_isaac_bridge ur5e_isaac.launch.py launch_isaac:=false
```

The ROS terminal and Isaac Sim need the same `ROS_DOMAIN_ID`. The bridge graph uses the domain from that environment variable.
