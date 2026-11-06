#!/usr/bin/env python3
import json
import os
import sys
import time
import cv2
import numpy as np
import yaml
from cv_bridge import CvBridge

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image, JointState
import tf2_ros
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class DatasetCollector(Node):
    def __init__(self, output_dir="dataset"):
        super().__init__("dataset_collector")
        self.output_dir = output_dir
        os.makedirs(self.output_dir, exist_ok=True)
        self.bridge = CvBridge()

        # Data buffers
        self.latest_image = None
        self.latest_camera_info = None
        self.latest_joint_state = None

        # TF listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Subscriptions
        self.create_subscription(Image, "/wrist_cam/image_raw", self._image_cb, 10)
        self.create_subscription(CameraInfo, "/wrist_cam/camera_info", self._info_cb, 10)
        self.create_subscription(JointState, "/joint_states", self._joint_cb, 10)

        # Trajectory publisher to move the arm
        self.traj_pub = self.create_publisher(
            JointTrajectory, "/joint_trajectory_controller/joint_trajectory", 10
        )

        self.joint_names = [
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint",
        ]

    def _image_cb(self, msg):
        self.latest_image = msg

    def _info_cb(self, msg):
        self.latest_camera_info = msg

    def _joint_cb(self, msg):
        self.latest_joint_state = msg

    def wait_for_data(self):
        self.get_logger().info("Waiting for camera and joint state streams...")
        while rclpy.ok() and (self.latest_image is None or self.latest_camera_info is None or self.latest_joint_state is None):
            rclpy.spin_once(self, timeout_sec=0.2)
        self.get_logger().info("Streams received!")

    def save_camera_info(self):
        info = self.latest_camera_info
        calib_data = {
            "image_width": int(info.width),
            "image_height": int(info.height),
            "camera_name": "wrist_cam",
            "camera_matrix": {
                "rows": 3,
                "cols": 3,
                "data": [float(x) for x in info.k],
            },
            "distortion_model": str(info.distortion_model),
            "distortion_coefficients": {
                "rows": 1,
                "cols": len(info.d),
                "data": [float(x) for x in info.d],
            },
            "rectification_matrix": {
                "rows": 3,
                "cols": 3,
                "data": [float(x) for x in info.r],
            },
            "projection_matrix": {
                "rows": 3,
                "cols": 4,
                "data": [float(x) for x in info.p],
            },
        }
        yaml_path = os.path.join(self.output_dir, "camera_info.yaml")
        with open(yaml_path, "w") as f:
            yaml.dump(calib_data, f, default_flow_style=False)
        self.get_logger().info(f"Saved camera info to {yaml_path}")

    def get_camera_pose(self):
        """Lookup transform from base_link to camera_optical_frame."""
        try:
            # Try camera_optical_frame, fall back to camera_optical_link if not renamed
            target_frame = "camera_optical_frame"
            if not self.tf_buffer.can_transform("base_link", target_frame, rclpy.time.Time()):
                target_frame = "camera_optical_link"

            t = self.tf_buffer.lookup_transform(
                "base_link", target_frame, rclpy.time.Time(), timeout=rclpy.duration.Duration(seconds=1.0)
            )
            return {
                "parent_frame": "base_link",
                "child_frame": target_frame,
                "translation": [
                    t.transform.translation.x,
                    t.transform.translation.y,
                    t.transform.translation.z,
                ],
                "rotation": [
                    t.transform.rotation.x,
                    t.transform.rotation.y,
                    t.transform.rotation.z,
                    t.transform.rotation.w,
                ],
            }
        except Exception as e:
            self.get_logger().error(f"TF lookup failed: {e}")
            return None

    def send_joint_goal(self, positions, duration_sec=4.0):
        traj = JointTrajectory()
        traj.joint_names = self.joint_names
        point = JointTrajectoryPoint()
        point.positions = positions
        point.time_from_start.sec = int(duration_sec)
        point.time_from_start.nanosec = int((duration_sec % 1) * 1e9)
        traj.points.append(point)
        self.traj_pub.publish(traj)

    def capture_pose(self, pose_idx):
        # Allow arm to settle and get fresh frame
        for _ in range(10):
            rclpy.spin_once(self, timeout_sec=0.1)

        # 1. Save Image
        cv_img = self.bridge.imgmsg_to_cv2(self.latest_image, desired_encoding="bgr8")
        img_filename = f"image_{pose_idx}.png"
        img_path = os.path.join(self.output_dir, img_filename)
        cv2.imwrite(img_path, cv_img)

        # 2. Get Joint Angles
        joint_dict = {}
        for name, pos in zip(self.latest_joint_state.name, self.latest_joint_state.position):
            if name in self.joint_names:
                joint_dict[name] = float(pos)

        # 3. Get Camera Pose
        cam_pose = self.get_camera_pose()

        entry = {
            "pose_id": pose_idx,
            "image": img_filename,
            "joint_angles": joint_dict,
            "camera_pose": cam_pose,
        }
        self.get_logger().info(f"[Pose {pose_idx}] Captured {img_filename}")
        return entry


def main():
    rclpy.init()
    collector = DatasetCollector()
    collector.wait_for_data()
    collector.save_camera_info()

    # 5 user-specified poses in degrees [pan, lift, elbow, wrist_1, wrist_2, wrist_3]
    # Note: Pose 5 uses 70 deg for wrist_3 (adjust if needed)
    poses_deg = [
        ("Pose 1", [-19, -12, -66, -151, 65, 94]),
        ("Pose 2", [-56, -9, -67, -139, 86, 91]),
        ("Pose 3", [-92, -28, -94, -107, 99, 72]),
        ("Pose 4", [-110, -52, -57, -132, 106, 55]),
        ("Pose 5", [-85, -23, -110, -96, 95, 77]),
    ]
    predefined_poses = [
        (name, [float(np.radians(deg)) for deg in angles])
        for name, angles in poses_deg
    ]

    poses_log = []
    print("\nSelect Mode:")
    print("  1: Automatic (moves through 5 predefined joint poses)")
    print("  2: Interactive (move arm yourself in RViz MoveIt and press Enter)\n")
    mode = input("Enter choice [1/2] (default 1): ").strip() or "1"

    if mode == "1":
        for i, (name, goal) in enumerate(predefined_poses):
            print(f"\nMoving to Pose {i} ({name})...")
            collector.send_joint_goal(goal, duration_sec=3.5)
            # Wait for movement + settling time
            time.sleep(4.5)
            poses_log.append(collector.capture_pose(i))
    else:
        for i in range(5):
            input(f"\nMove arm in RViz to Pose {i} looking at the cube, then press [Enter] to capture...")
            poses_log.append(collector.capture_pose(i))

    # Save poses.jsonl
    jsonl_path = os.path.join(collector.output_dir, "poses.jsonl")
    with open(jsonl_path, "w") as f:
        for entry in poses_log:
            f.write(json.dumps(entry) + "\n")
    print(f"\nDataset complete! Saved records to {jsonl_path}")

    collector.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()