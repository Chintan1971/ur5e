# Spawn the official UR description into Isaac Sim and publish joint state
# over the ROS 2 bridge. Run with Isaac Sim's Python, for example:
#   ~/isaacsim/python.sh spawn_ur5e.py --xacro /path/ur_isaac.urdf.xacro

import argparse
import os
import subprocess
import sys
import tempfile

# Strip system ROS Python 3.10 paths from sys.path to ensure Isaac Sim's Python 3.11
# uses its internal/compatible ROS 2 bridge libraries without version conflicts.
sys.path = [p for p in sys.path if "python3.10" not in p]

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--xacro", required=True, help="Path to ur_isaac.urdf.xacro")
parser.add_argument("--ur-type", default="ur5e")
parser.add_argument("--joint-commands-topic", default="/isaac_joint_commands")
parser.add_argument("--joint-states-topic", default="/isaac_joint_states")
parser.add_argument("--headless", action="store_true")
args, _unknown = parser.parse_known_args()

from isaacsim import SimulationApp

simulation_app = SimulationApp(
    {"renderer": "RealTimePathTracing", "headless": args.headless}
)

import numpy as np
import omni.graph.core as og
import usdrt.Sdf
from isaacsim.core.simulation_manager import SimulationManager
from isaacsim.storage.native import get_assets_root_path
from pxr import Usd, UsdPhysics

# Compatibility layer across Isaac Sim versions (5.x and 6.x)
try:
    from isaacsim.core.experimental.utils import app as app_utils
except ImportError:
    import omni.timeline
    from isaacsim.core.utils.extensions import enable_extension

    class _AppUtils:
        @staticmethod
        def enable_extension(name: str):
            enable_extension(name)

        @staticmethod
        def play():
            omni.timeline.get_timeline_interface().play()

        @staticmethod
        def stop():
            omni.timeline.get_timeline_interface().stop()

    app_utils = _AppUtils()

try:
    from isaacsim.core.experimental.utils import stage as stage_utils
except ImportError:
    from isaacsim.core.utils import stage as stage_utils

try:
    from isaacsim.core.rendering_manager import ViewportManager

    def set_camera_view(eye, target):
        ViewportManager.set_camera_view("/OmniverseKit_Persp", eye=eye, target=target)
except ImportError:
    from isaacsim.core.utils.viewports import set_camera_view as _scv

    def set_camera_view(eye, target):
        _scv(eye=eye, target=target, camera_prim_path="/OmniverseKit_Persp")

try:
    from isaacsim.asset.importer.urdf import URDFImporter, URDFImporterConfig
    _has_urdf_importer_class = True
except ImportError:
    _has_urdf_importer_class = False
    import omni.kit.commands
    from isaacsim.asset.importer.urdf import _urdf


def expand_urdf() -> str:
    """Expand the official xacro with absolute mesh paths for the importer."""
    out_dir = tempfile.mkdtemp(prefix="ur5e_isaac_urdf_")
    urdf_path = os.path.join(out_dir, "ur5e.urdf")
    command = [
        "xacro",
        args.xacro,
        f"ur_type:={args.ur_type}",
        "name:=ur",
        "force_abs_paths:=true",
        f"joint_commands_topic:={args.joint_commands_topic}",
        f"joint_states_topic:={args.joint_states_topic}",
    ]
    # Build a clean environment for xacro (which uses system Python 3.10)
    xacro_env = os.environ.copy()
    distro = os.environ.get("ROS_DISTRO", "humble")
    ros_python_paths = [
        p for p in os.environ.get("PYTHONPATH", "").split(":")
        if p and "kit" not in p and "3.11" not in p and "miniconda" not in p
    ]
    if not any("site-packages" in p for p in ros_python_paths):
        ros_python_paths.append(f"/opt/ros/{distro}/lib/python3.10/site-packages")
        ros_python_paths.append(f"/opt/ros/{distro}/local/lib/python3.10/dist-packages")
    xacro_env["PYTHONPATH"] = ":".join(ros_python_paths)

    # Derive workspace install path from args.xacro
    xacro_abs = os.path.abspath(args.xacro)
    if "/install/" in xacro_abs:
        ws_install = xacro_abs.split("/install/")[0] + "/install"
        ament_prefixes = [
            os.path.join(ws_install, "ur_description"),
            os.path.join(ws_install, "ur_moveit_config"),
            os.path.join(ws_install, "ur5e_isaac_bridge"),
        ]
        curr_ament = [p for p in os.environ.get("AMENT_PREFIX_PATH", "").split(":") if p]
        all_ament = ament_prefixes + [p for p in curr_ament if p not in ament_prefixes]
        if f"/opt/ros/{distro}" not in all_ament:
            all_ament.append(f"/opt/ros/{distro}")
        xacro_env["AMENT_PREFIX_PATH"] = ":".join(all_ament)

    result = subprocess.run(command, env=xacro_env, check=False, text=True, capture_output=True)
    if result.returncode != 0:
        sys.stderr.write(result.stderr)
        raise RuntimeError(
            f"xacro failed:\n{result.stderr}\nSource the ROS 2 workspace before launching Isaac Sim."
        )
    with open(urdf_path, "w", encoding="utf-8") as urdf_file:
        urdf_file.write(result.stdout)
    return urdf_path


def articulation_path(hint: str) -> str:
    if hasattr(stage_utils, "get_current_stage"):
        try:
            stage = stage_utils.get_current_stage(backend="usd")
        except TypeError:
            stage = stage_utils.get_current_stage()
    else:
        stage = omni.usd.get_context().get_stage()

    prim = stage.GetPrimAtPath(hint)
    if prim and prim.IsValid() and prim.HasAPI(UsdPhysics.ArticulationRootAPI):
        return hint
    if prim and prim.IsValid():
        for child in Usd.PrimRange(prim):
            if child.HasAPI(UsdPhysics.ArticulationRootAPI):
                return str(child.GetPath())
    return hint


def import_robot_to_usd(urdf_path: str) -> str:
    usd_dir = tempfile.mkdtemp(prefix="ur5e_isaac_usd_")
    usd_path = os.path.join(usd_dir, "ur5e.usd")
    if _has_urdf_importer_class:
        importer = URDFImporter(
            URDFImporterConfig(
                urdf_path=urdf_path,
                usd_path=usd_dir,
                fix_base=True,
                merge_fixed_joints=False,
                collision_from_visuals=False,
                joint_drive_type="force",
                joint_target_type="position",
                override_joint_stiffness=100000.0,
                override_joint_damping=10000.0,
                run_asset_transformer=False,
            )
        )
        return importer.import_urdf()
    else:
        import_config = _urdf.ImportConfig()
        import_config.merge_fixed_joints = False
        import_config.fix_base = True
        import_config.make_default_prim = True
        import_config.self_collision = False
        import_config.collision_from_visuals = False
        import_config.default_drive_type = _urdf.UrdfJointTargetType.JOINT_DRIVE_POSITION
        import_config.default_drive_strength = 100000.0
        import_config.default_position_drive_damping = 10000.0

        status, imported_robot = omni.kit.commands.execute(
            "URDFParseFile",
            urdf_path=urdf_path,
            import_config=import_config,
        )
        status, stage_path = omni.kit.commands.execute(
            "URDFImportRobot",
            urdf_path=urdf_path,
            urdf_robot=imported_robot,
            import_config=import_config,
            dest_path=usd_path,
        )
        return usd_path


def main() -> None:
    app_utils.enable_extension("isaacsim.ros2.bridge")
    app_utils.enable_extension("isaacsim.core.nodes")
    app_utils.enable_extension("isaacsim.asset.importer.urdf")
    simulation_app.update()

    stage_utils.set_stage_units(meters_per_unit=1.0)
    set_camera_view(
        eye=np.array([1.6, 1.6, 1.2]),
        target=np.array([0.0, 0.0, 0.4]),
    )

    assets_root = get_assets_root_path()
    if assets_root:
        stage_utils.add_reference_to_stage(
            assets_root + "/Isaac/Environments/Simple_Room/simple_room.usd",
            "/background",
        )

    urdf_path = expand_urdf()
    usd_path = import_robot_to_usd(urdf_path)
    robot_root = "/ur5e"
    stage_utils.add_reference_to_stage(usd_path, robot_root)
    simulation_app.update()
    robot_path = articulation_path(robot_root)
    print(f"UR5e articulation: {robot_path}")

    # -------------------------------------------------------------
    # Spawn Dynamic Cuboid in front of the UR5e base
    # -------------------------------------------------------------
    from isaacsim.core.api.objects import DynamicCuboid
    cube = DynamicCuboid(
        prim_path="/World/Cube",
        name="target_cube",
        position=np.array([0.0, 0.5, 0.025]),   # 0.5m in front of base (+X), resting on ground
        size=0.1,                              # 5 cm edge length
        color=np.array([0.8, 0.1, 0.1]),        # Red color
        mass=0.1,                               # 100 grams
    )

    from isaacsim.sensors.camera import Camera

    gripper_cam = Camera(
    prim_path="/ur5e/camera_link/Camera",       
    resolution=(640, 480),
    frequency=15,                              
    translation=np.array([0.015, 0.0, 0.0]),
    )
    gripper_cam.initialize()
    gripper_cam.set_clipping_range(near_distance=0.01, far_distance=1000.0)

    render_product_path = gripper_cam.get_render_product_path()

    commands_topic = args.joint_commands_topic.lstrip("/")
    states_topic = args.joint_states_topic.lstrip("/")

    # Check if IsaacReadJointState node is available (Isaac Sim 6.0+) or direct targetPrim (Isaac Sim 5.x)
    _read_node = og.get_node_type("isaacsim.sensors.physics.IsaacReadJointState")
    has_read_joint_state = bool(_read_node and _read_node.is_valid())

    if has_read_joint_state:
        app_utils.enable_extension("isaacsim.sensors.physics")
        create_nodes = [
            ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
            ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
            ("ReadJointState", "isaacsim.sensors.physics.IsaacReadJointState"),
            ("Context", "isaacsim.ros2.bridge.ROS2Context"),
            ("PublishJointState", "isaacsim.ros2.bridge.ROS2PublishJointState"),
            ("SubscribeJointState", "isaacsim.ros2.bridge.ROS2SubscribeJointState"),
            ("ArticulationController", "isaacsim.core.nodes.IsaacArticulationController"),
            ("PublishClock", "isaacsim.ros2.bridge.ROS2PublishClock"),
        ]
        connect_edges = [
            ("OnPlaybackTick.outputs:tick", "ReadJointState.inputs:execIn"),
            ("ReadJointState.outputs:execOut", "PublishJointState.inputs:execIn"),
            ("ReadJointState.outputs:jointNames", "PublishJointState.inputs:jointNames"),
            ("ReadJointState.outputs:jointPositions", "PublishJointState.inputs:jointPositions"),
            ("ReadJointState.outputs:jointVelocities", "PublishJointState.inputs:jointVelocities"),
            ("ReadJointState.outputs:jointEfforts", "PublishJointState.inputs:jointEfforts"),
            ("ReadJointState.outputs:jointDofTypes", "PublishJointState.inputs:jointDofTypes"),
            ("ReadJointState.outputs:stageMetersPerUnit", "PublishJointState.inputs:stageMetersPerUnit"),
            ("ReadJointState.outputs:sensorTime", "PublishJointState.inputs:sensorTime"),
            ("OnPlaybackTick.outputs:tick", "SubscribeJointState.inputs:execIn"),
            ("OnPlaybackTick.outputs:tick", "PublishClock.inputs:execIn"),
            ("OnPlaybackTick.outputs:tick", "ArticulationController.inputs:execIn"),
            ("Context.outputs:context", "PublishJointState.inputs:context"),
            ("Context.outputs:context", "SubscribeJointState.inputs:context"),
            ("Context.outputs:context", "PublishClock.inputs:context"),
            ("ReadSimTime.outputs:simulationTime", "PublishClock.inputs:timeStamp"),
            ("SubscribeJointState.outputs:jointNames", "ArticulationController.inputs:jointNames"),
            (
                "SubscribeJointState.outputs:positionCommand",
                "ArticulationController.inputs:positionCommand",
            ),
            (
                "SubscribeJointState.outputs:velocityCommand",
                "ArticulationController.inputs:velocityCommand",
            ),
            (
                "SubscribeJointState.outputs:effortCommand",
                "ArticulationController.inputs:effortCommand",
            ),
        ]
        set_values = [
            ("ArticulationController.inputs:robotPath", robot_path),
            ("ReadJointState.inputs:prim", [usdrt.Sdf.Path(robot_path)]),
            ("PublishJointState.inputs:topicName", states_topic),
            ("SubscribeJointState.inputs:topicName", commands_topic),
        ]
    else:
        create_nodes = [
            ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
            ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
            ("Context", "isaacsim.ros2.bridge.ROS2Context"),
            ("PublishJointState", "isaacsim.ros2.bridge.ROS2PublishJointState"),
            ("SubscribeJointState", "isaacsim.ros2.bridge.ROS2SubscribeJointState"),
            ("ArticulationController", "isaacsim.core.nodes.IsaacArticulationController"),
            ("PublishClock", "isaacsim.ros2.bridge.ROS2PublishClock"),
        ]
        connect_edges = [
            ("OnPlaybackTick.outputs:tick", "PublishJointState.inputs:execIn"),
            ("OnPlaybackTick.outputs:tick", "SubscribeJointState.inputs:execIn"),
            ("OnPlaybackTick.outputs:tick", "PublishClock.inputs:execIn"),
            ("OnPlaybackTick.outputs:tick", "ArticulationController.inputs:execIn"),
            ("Context.outputs:context", "PublishJointState.inputs:context"),
            ("Context.outputs:context", "SubscribeJointState.inputs:context"),
            ("Context.outputs:context", "PublishClock.inputs:context"),
            ("ReadSimTime.outputs:simulationTime", "PublishJointState.inputs:timeStamp"),
            ("ReadSimTime.outputs:simulationTime", "PublishClock.inputs:timeStamp"),
            ("SubscribeJointState.outputs:jointNames", "ArticulationController.inputs:jointNames"),
            (
                "SubscribeJointState.outputs:positionCommand",
                "ArticulationController.inputs:positionCommand",
            ),
            (
                "SubscribeJointState.outputs:velocityCommand",
                "ArticulationController.inputs:velocityCommand",
            ),
            ("SubscribeJointState.outputs:effortCommand", "ArticulationController.inputs:effortCommand"),
        ]
        set_values = [
            ("ArticulationController.inputs:robotPath", robot_path),
            ("PublishJointState.inputs:topicName", states_topic),
            ("SubscribeJointState.inputs:topicName", commands_topic),
            ("PublishJointState.inputs:targetPrim", [usdrt.Sdf.Path(robot_path)]),
        ]

    # -------------------------------------------------------------
    # Wrist Camera ROS 2 Publishing (RGB + CameraInfo)
    # -------------------------------------------------------------
    create_nodes.extend([
        ("CameraRGBPublish", "isaacsim.ros2.bridge.ROS2CameraHelper"),
        ("CameraInfoPublish", "isaacsim.ros2.bridge.ROS2CameraInfoHelper"),
    ])
    connect_edges.extend([
        ("OnPlaybackTick.outputs:tick", "CameraRGBPublish.inputs:execIn"),
        ("OnPlaybackTick.outputs:tick", "CameraInfoPublish.inputs:execIn"),
        ("Context.outputs:context", "CameraRGBPublish.inputs:context"),
        ("Context.outputs:context", "CameraInfoPublish.inputs:context"),
    ])
    set_values.extend([
        # RGB Image Publisher -> /wrist_cam/image_raw
        ("CameraRGBPublish.inputs:renderProductPath", render_product_path),
        ("CameraRGBPublish.inputs:topicName", "wrist_cam/image_raw"),
        ("CameraRGBPublish.inputs:type", "rgb"),
        ("CameraRGBPublish.inputs:frameId", "camera_optical_link"),
        ("CameraRGBPublish.inputs:frameSkipCount", 3),       # 60 Hz / (3 + 1) = 15 Hz
        ("CameraRGBPublish.inputs:useSystemTime", False),    #  simulation time (/clock)
        ("CameraRGBPublish.inputs:resetSimulationTimeOnStop", True),
        # Camera Info Publisher -> /wrist_cam/camera_info
        ("CameraInfoPublish.inputs:renderProductPath", render_product_path),
        ("CameraInfoPublish.inputs:topicName", "wrist_cam/camera_info"),
        ("CameraInfoPublish.inputs:frameId", "camera_optical_link"),
        ("CameraInfoPublish.inputs:frameSkipCount", 3),       # Keep synchronized with RGB at 15 Hz
        ("CameraInfoPublish.inputs:useSystemTime", False),    # simulation time (/clock)
        ("CameraInfoPublish.inputs:resetSimulationTimeOnStop", True),
    ])


    og.Controller.edit(
        {"graph_path": "/ActionGraph", "evaluator_name": "execution"},
        {
            og.Controller.Keys.CREATE_NODES: create_nodes,
            og.Controller.Keys.CONNECT: connect_edges,
            og.Controller.Keys.SET_VALUES: set_values,
        },
    )

    # Optional configuration: match environment ROS domain ID and topic name for clock
    for attr_path, value in (
        ("/ActionGraph/Context.inputs:useDomainIDEnvVar", True),
        ("/ActionGraph/PublishClock.inputs:topicName", "clock"),
    ):
        try:
            og.Controller.set(og.Controller.attribute(attr_path), value)
        except Exception as exc:
            print(f"Left {attr_path} at its default: {exc}")

    simulation_app.update()
    if hasattr(SimulationManager, "setup_simulation"):
        SimulationManager.setup_simulation(dt=1.0 / 60.0, device="cpu")
    else:
        try:
            from isaacsim.core.api import PhysicsContext
            PhysicsContext(physics_dt=1.0 / 60.0)
        except Exception:
            SimulationManager.set_physics_dt(1.0 / 60.0)
            SimulationManager.initialize_physics()
    app_utils.play()
    simulation_app.update()

    while simulation_app.is_running():
        simulation_app.update()

    app_utils.stop()
    simulation_app.close()


if __name__ == "__main__":
    main()
