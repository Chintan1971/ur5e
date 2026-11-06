"""Launch a UR5e into Isaac Sim and bring up MoveIt against the ROS 2 bridge.

    source /opt/ros/humble/setup.bash
    source install/setup.bash
    ros2 launch ur5e_isaac_bridge ur5e_isaac.launch.py

The standalone script starts Isaac Sim, imports the official UR description,
and publishes /clock and /isaac_joint_states. MoveIt starts after those
topics are available.
"""

from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils import MoveItConfigsBuilder


def _bridge_share() -> str:
    return get_package_share_directory("ur5e_isaac_bridge")


def _wait_for_bridge(states_topic: str) -> ExecuteProcess:
    return ExecuteProcess(
        cmd=[
            "bash",
            "-c",
            f"""
set -euo pipefail
echo "[ur5e_isaac] Waiting for Isaac ROS 2 bridge: /clock and {states_topic} (180s)..."
deadline=$((SECONDS + 180))
while (( SECONDS < deadline )); do
  if ros2 topic echo /clock --once >/dev/null 2>&1 \
     && ros2 topic echo {states_topic} --once >/dev/null 2>&1; then
    echo "[ur5e_isaac] Bridge is publishing."
    exit 0
  fi
  sleep 0.5
done
echo "[ur5e_isaac] Timed out waiting for the Isaac Sim ROS 2 bridge." >&2
exit 1
""",
        ],
        output="screen",
    )


def _ros_stack(context):
    ur_type = LaunchConfiguration("ur_type").perform(context)
    use_sim_time = LaunchConfiguration("use_sim_time").perform(context) == "true"
    commands_topic = LaunchConfiguration("joint_commands_topic").perform(context)
    states_topic = LaunchConfiguration("joint_states_topic").perform(context)

    bridge_share = _bridge_share()
    xacro_path = os.path.join(bridge_share, "urdf", "ur_isaac.urdf.xacro")
    controllers_yaml = os.path.join(bridge_share, "config", "ros2_controllers.yaml")
    moveit_controllers = os.path.join(bridge_share, "config", "moveit_controllers.yaml")
    rviz_config = os.path.join(
        get_package_share_directory("ur_moveit_config"), "config", "moveit.rviz"
    )

    moveit_config = (
        MoveItConfigsBuilder("ur", package_name="ur_moveit_config")
        .robot_description(
            file_path=xacro_path,
            mappings={
                "ur_type": ur_type,
                "name": "ur",
                "force_abs_paths": "true",
                "joint_commands_topic": commands_topic,
                "joint_states_topic": states_topic,
            },
        )
        .robot_description_semantic(
            file_path="srdf/ur.srdf.xacro",
            mappings={"name": "ur"},
        )
        .trajectory_execution(file_path=moveit_controllers)
        .to_moveit_configs()
    )

    if os.environ.get("ROS_DISTRO", "humble") == "humble":
        for pipeline_name, p in moveit_config.planning_pipelines.items():
            if isinstance(p, dict):
                p.pop("response_adapters", None)
                if isinstance(p.get("request_adapters"), list):
                    if pipeline_name == "pilz_industrial_motion_planner":
                        p["request_adapters"] = ""
                    else:
                        adapters = [
                            a.replace("default_planning_request_adapters", "default_planner_request_adapters")
                            for a in p["request_adapters"]
                        ]
                        p["request_adapters"] = " ".join(adapters)
                if "planning_plugins" in p and isinstance(p["planning_plugins"], list):
                    p["planning_plugin"] = p["planning_plugins"][0]

    robot_description = {
        "robot_description": ParameterValue(
            moveit_config.robot_description["robot_description"], value_type=str
        )
    }

    return [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            parameters=[robot_description, {"use_sim_time": use_sim_time}],
        ),
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            output="screen",
            parameters=[
                robot_description,
                controllers_yaml,
                {"use_sim_time": use_sim_time},
            ],
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            output="screen",
            arguments=[
                "joint_state_broadcaster",
                "joint_trajectory_controller",
                "--controller-manager",
                "/controller_manager",
            ],
        ),
        Node(
            package="moveit_ros_move_group",
            executable="move_group",
            output="screen",
            parameters=[
                moveit_config.to_dict(),
                {
                    "use_sim_time": use_sim_time,
                    "publish_robot_description_semantic": True,
                },
            ],
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2_moveit",
            output="log",
            arguments=["-d", rviz_config],
            parameters=[
                moveit_config.robot_description,
                moveit_config.robot_description_semantic,
                moveit_config.robot_description_kinematics,
                moveit_config.planning_pipelines,
                moveit_config.joint_limits,
                {"use_sim_time": use_sim_time},
            ],
            condition=IfCondition(LaunchConfiguration("launch_rviz")),
        ),
    ]


def _start(context, *args, **kwargs):
    del args, kwargs
    states_topic = LaunchConfiguration("joint_states_topic").perform(context)
    cmd = [
        LaunchConfiguration("isaac_python").perform(context),
        os.path.join(_bridge_share(), "scripts", "spawn_ur5e.py"),
        "--xacro",
        os.path.join(_bridge_share(), "urdf", "ur_isaac.urdf.xacro"),
        "--ur-type",
        LaunchConfiguration("ur_type").perform(context),
        "--joint-commands-topic",
        LaunchConfiguration("joint_commands_topic").perform(context),
        "--joint-states-topic",
        states_topic,
    ]
    if LaunchConfiguration("headless").perform(context) == "true":
        cmd.append("--headless")

    actions = []
    if LaunchConfiguration("launch_isaac").perform(context) == "true":
        distro = os.environ.get("ROS_DISTRO", "humble")
        isaac_env = {
            "ROS_DISTRO": distro,
            "RMW_IMPLEMENTATION": os.environ.get("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp"),
        }
        isaac_bin = LaunchConfiguration("isaac_python").perform(context)
        isaac_path = os.path.dirname(os.path.realpath(isaac_bin))
        bridge_lib = os.path.join(isaac_path, "exts", "isaacsim.ros2.bridge", distro, "lib")
        current_ld = os.environ.get("LD_LIBRARY_PATH", "")
        if os.path.isdir(bridge_lib):
            isaac_env["LD_LIBRARY_PATH"] = f"{bridge_lib}:{current_ld}" if current_ld else bridge_lib

        # Remove conda environments from PATH to avoid conflicts with Isaac Sim internal Python
        current_path = os.environ.get("PATH", "")
        clean_path = ":".join([p for p in current_path.split(":") if "miniconda" not in p and "conda" not in p])
        if clean_path:
            isaac_env["PATH"] = clean_path

        actions.append(
            ExecuteProcess(
                cmd=cmd,
                additional_env=isaac_env,
                output="screen",
            )
        )

    wait = _wait_for_bridge(states_topic)

    def _on_bridge(event, launch_context):
        if event.returncode != 0:
            return [
                LogInfo(msg="Isaac Sim bridge did not start. MoveIt was not launched.")
            ]
        return _ros_stack(launch_context)

    actions.append(wait)
    actions.append(
        RegisterEventHandler(OnProcessExit(target_action=wait, on_exit=_on_bridge))
    )
    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("ur_type", default_value="ur5e"),
            DeclareLaunchArgument("launch_isaac", default_value="true"),
            DeclareLaunchArgument("launch_rviz", default_value="true"),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("headless", default_value="false"),
            DeclareLaunchArgument(
                "isaac_python",
                default_value=os.path.expanduser("~/isaacsim/python.sh"),
            ),
            DeclareLaunchArgument(
                "joint_commands_topic", default_value="/isaac_joint_commands"
            ),
            DeclareLaunchArgument(
                "joint_states_topic", default_value="/isaac_joint_states"
            ),
            SetParameter(name="use_sim_time", value=LaunchConfiguration("use_sim_time")),
            OpaqueFunction(function=_start),
        ]
    )
