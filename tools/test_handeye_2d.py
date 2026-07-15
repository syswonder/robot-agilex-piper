#!/usr/bin/env python3
"""Click an RGB pixel, map it through 2D hand-eye, then execute via Robonix."""

from __future__ import annotations

import argparse
import math
import os
import sys
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import Pose, Quaternion
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from std_srvs.srv import Trigger


REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_HOMOGRAPHY = REPO_ROOT / "rbnx-boot" / "hand-eye-data" / "2d_homography.npy"
DEFAULT_CODEGEN = (
    REPO_ROOT
    / "rbnx-boot"
    / "cache"
    / "roboarm_ik_rbnx"
    / "rbnx-build"
    / "codegen"
    / "proto_gen"
)
EXECUTE_GRASP_CONTRACT = "robonix/service/manipulation/execute_grasp"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Move to the observation pose, show a live RGB image, map a clicked "
            "pixel through the 2D homography, and call the active Robonix "
            "manipulation/execute_grasp provider."
        )
    )
    parser.add_argument(
        "--homography",
        default=str(DEFAULT_HOMOGRAPHY),
        help="3x3 .npy homography mapping image pixel [u,v,1] to arm/base_link XY.",
    )
    parser.add_argument(
        "--image-topic",
        default="/camera/color/image_raw",
        help="ROS Image topic to display.",
    )
    parser.add_argument(
        "--reset-service",
        default="/moveit_control/reset",
        help="std_srvs/Trigger service that parks the arm at the observation pose.",
    )
    parser.add_argument(
        "--frame-id",
        default="arm/base_link",
        help="Frame id for the target pose.",
    )
    parser.add_argument(
        "--z",
        type=float,
        default=-0.20,
        help="Target grasp z in frame-id, meters.",
    )
    parser.add_argument(
        "--yaw",
        type=float,
        default=0.0,
        help="Vertical-grasp yaw in radians, used only with --orientation-mode vertical.",
    )
    parser.add_argument(
        "--orientation-mode",
        choices=("current", "vertical-current-yaw", "vertical"),
        default="current",
        help=(
            "current keeps the end-effector orientation observed after reset; "
            "vertical-current-yaw keeps that yaw but forces a vertical-down grasp; "
            "vertical uses a fixed vertical-down grasp quaternion."
        ),
    )
    parser.add_argument(
        "--end-pose-topic",
        default="/arm/end_pose",
        help="Pose topic used by --orientation-mode current.",
    )
    parser.add_argument(
        "--gripper-width",
        type=float,
        default=0.04,
        help="Gripper width in meters.",
    )
    parser.add_argument(
        "--execute-timeout",
        type=float,
        default=20.0,
        help="Seconds allowed for manipulation/execute_grasp.",
    )
    parser.add_argument(
        "--image-timeout",
        type=float,
        default=10.0,
        help="Seconds to wait for an image.",
    )
    parser.add_argument(
        "--reset-timeout",
        type=float,
        default=60.0,
        help="Seconds to wait for reset service and response.",
    )
    parser.add_argument(
        "--resolve-timeout",
        type=float,
        default=20.0,
        help="Seconds to wait for atlas to resolve execute_grasp.",
    )
    parser.add_argument(
        "--atlas",
        default=os.environ.get("ROBONIX_ATLAS", "127.0.0.1:50051"),
        help="Atlas gRPC endpoint.",
    )
    parser.add_argument(
        "--codegen-dir",
        default=str(DEFAULT_CODEGEN),
        help="Directory containing generated atlas_pb2.py and atlas_pb2_grpc.py.",
    )
    parser.add_argument(
        "--skip-reset",
        action="store_true",
        help="Do not call reset before showing the image.",
    )
    return parser.parse_args()


def image_to_robot_xy(homography: np.ndarray, u: float, v: float) -> tuple[float, float]:
    projected = homography @ np.array([u, v, 1.0], dtype=np.float64)
    if abs(float(projected[2])) < 1e-9:
        raise ValueError("homography projection has near-zero scale")
    projected = projected / projected[2]
    return float(projected[0]), float(projected[1])


def vertical_quaternion(yaw_rad: float) -> tuple[float, float, float, float]:
    half_yaw = yaw_rad * 0.5
    # Same convention as roboarm:
    # R.from_euler("zyx", [degrees(yaw), 180.0, 0.0], degrees=True)
    return math.sin(half_yaw), math.cos(half_yaw), 0.0, 0.0


def quaternion_to_yaw(q: Quaternion) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def ros_image_to_bgr(msg: Image) -> np.ndarray:
    dtype = np.uint16 if msg.encoding in ("16UC1", "mono16") else np.uint8
    channels_by_encoding = {
        "rgb8": 3,
        "bgr8": 3,
        "rgba8": 4,
        "bgra8": 4,
        "mono8": 1,
        "8UC1": 1,
        "16UC1": 1,
        "mono16": 1,
    }
    channels = channels_by_encoding.get(msg.encoding)
    if channels is None:
        raise ValueError(f"unsupported image encoding: {msg.encoding!r}")

    data = np.frombuffer(msg.data, dtype=dtype)
    row_elems = msg.step // np.dtype(dtype).itemsize
    image = data.reshape((msg.height, row_elems))
    image = image[:, : msg.width * channels]
    if channels > 1:
        image = image.reshape((msg.height, msg.width, channels))
    else:
        image = image.reshape((msg.height, msg.width))

    if msg.encoding == "rgb8":
        return cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    if msg.encoding == "rgba8":
        return cv2.cvtColor(image, cv2.COLOR_RGBA2BGR)
    if msg.encoding == "bgra8":
        return cv2.cvtColor(image, cv2.COLOR_BGRA2BGR)
    if msg.encoding in ("mono8", "8UC1"):
        return cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
    if msg.encoding in ("16UC1", "mono16"):
        scaled = cv2.convertScaleAbs(image, alpha=255.0 / max(float(image.max()), 1.0))
        return cv2.cvtColor(scaled, cv2.COLOR_GRAY2BGR)
    return image.copy()


class Handeye2DTestNode(Node):
    def __init__(self, args: argparse.Namespace):
        super().__init__("handeye_2d_test")
        self.args = args
        self.latest_image: Image | None = None
        self.latest_end_pose: Pose | None = None
        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT
        qos.durability = DurabilityPolicy.VOLATILE
        self.create_subscription(Image, args.image_topic, self._image_cb, qos)
        self.create_subscription(Pose, args.end_pose_topic, self._end_pose_cb, 10)
        self.reset_client = self.create_client(Trigger, args.reset_service)

    def _image_cb(self, msg: Image) -> None:
        self.latest_image = msg

    def _end_pose_cb(self, msg: Pose) -> None:
        self.latest_end_pose = msg

    def reset_to_observe_pose(self) -> None:
        if self.args.skip_reset:
            self.get_logger().info("skip reset; using current camera pose")
            return
        self.get_logger().info(f"waiting for reset service {self.args.reset_service}")
        if not self.reset_client.wait_for_service(timeout_sec=self.args.reset_timeout):
            raise TimeoutError(f"reset service not available: {self.args.reset_service}")
        self.get_logger().info("calling reset to move arm to observation pose")
        future = self.reset_client.call_async(Trigger.Request())
        deadline = time.monotonic() + self.args.reset_timeout
        while rclpy.ok() and not future.done() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        if not future.done():
            raise TimeoutError("reset service call timed out")
        resp = future.result()
        if resp is None or not resp.success:
            message = "" if resp is None else resp.message
            raise RuntimeError(f"reset failed: {message}")
        self.get_logger().info(f"reset ok: {resp.message}")

    def wait_for_image(self) -> Image:
        self.get_logger().info(f"waiting for image on {self.args.image_topic}")
        deadline = time.monotonic() + self.args.image_timeout
        while rclpy.ok() and self.latest_image is None and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        if self.latest_image is None:
            raise TimeoutError(f"no image on {self.args.image_topic}")
        return self.latest_image

    def wait_for_end_pose(self) -> Pose:
        self.get_logger().info(f"waiting for end pose on {self.args.end_pose_topic}")
        deadline = time.monotonic() + self.args.image_timeout
        while rclpy.ok() and self.latest_end_pose is None and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        if self.latest_end_pose is None:
            raise TimeoutError(f"no end pose on {self.args.end_pose_topic}")
        return self.latest_end_pose


def pick_pixel(node: Handeye2DTestNode) -> tuple[int, int] | None:
    selected: list[tuple[int, int]] = []
    window_name = "handeye_2d_test: click target, Enter/Space=execute, q/Esc=cancel"
    display = ros_image_to_bgr(node.latest_image)

    def on_mouse(event, x, y, _flags, _param):
        if event != cv2.EVENT_LBUTTONDOWN:
            return
        selected[:] = [(int(x), int(y))]

    cv2.namedWindow(window_name, cv2.WINDOW_AUTOSIZE)
    cv2.setMouseCallback(window_name, on_mouse)
    while True:
        rclpy.spin_once(node, timeout_sec=0.0)
        if node.latest_image is not None:
            display = ros_image_to_bgr(node.latest_image)
        frame = display.copy()
        if selected:
            cv2.circle(frame, selected[0], 6, (0, 0, 255), 2)
        cv2.imshow(window_name, frame)
        key = cv2.waitKey(30) & 0xFF
        if key in (13, 32) and selected:
            cv2.destroyWindow(window_name)
            return selected[0]
        if key in (27, ord("q")):
            cv2.destroyWindow(window_name)
            return None


def _load_atlas_modules(codegen_dir: str):
    path = Path(codegen_dir).expanduser().resolve()
    if not path.exists():
        raise RuntimeError(f"codegen dir does not exist: {path}")
    sys.path.insert(0, str(path))
    import atlas_pb2  # type: ignore
    import atlas_pb2_grpc  # type: ignore
    import grpc  # type: ignore
    import manipulation_pb2  # type: ignore
    import robonix_contracts_pb2_grpc  # type: ignore

    return grpc, atlas_pb2, atlas_pb2_grpc, manipulation_pb2, robonix_contracts_pb2_grpc


def _provider_has_contract(provider, contract_id: str, transport: int) -> bool:
    for cap in provider.capabilities:
        if cap.contract_id == contract_id and cap.transport == transport:
            return True
    return False


def resolve_execute_grasp(args: argparse.Namespace):
    grpc, atlas_pb2, atlas_pb2_grpc, manipulation_pb2, contracts_grpc = (
        _load_atlas_modules(args.codegen_dir))
    channel = grpc.insecure_channel(args.atlas)
    stub = atlas_pb2_grpc.AtlasStub(channel)
    transport = atlas_pb2.TRANSPORT_GRPC
    deadline = time.monotonic() + float(args.resolve_timeout)
    last_seen: list[str] = []

    while time.monotonic() < deadline:
        resp = stub.Query(
            atlas_pb2.QueryRequest(
                contract_id=EXECUTE_GRASP_CONTRACT,
                transport=transport,
            ),
            timeout=2.0,
        )
        providers = [
            provider
            for provider in resp.providers
            if provider.state == atlas_pb2.STATE_ACTIVE
            and _provider_has_contract(provider, EXECUTE_GRASP_CONTRACT, transport)
        ]
        last_seen = [provider.id for provider in resp.providers]
        if len(providers) == 1:
            provider = providers[0]
            connected = stub.ConnectCapability(
                atlas_pb2.ConnectCapabilityRequest(
                    consumer_id="test_handeye_2d",
                    provider_id=provider.id,
                    contract_id=EXECUTE_GRASP_CONTRACT,
                    transport=transport,
                ),
                timeout=5.0,
            )
            if not connected.endpoint:
                raise RuntimeError(f"atlas returned empty endpoint for {provider.id}")
            return (
                grpc,
                channel,
                stub,
                atlas_pb2,
                manipulation_pb2,
                contracts_grpc,
                connected.channel_id,
                connected.endpoint,
            )
        if len(providers) > 1:
            names = [provider.id for provider in providers]
            raise RuntimeError(f"multiple active execute_grasp providers: {names}")
        time.sleep(0.5)

    raise TimeoutError(
        f"atlas could not resolve active {EXECUTE_GRASP_CONTRACT!r} "
        f"within {args.resolve_timeout:.1f}s; seen providers={last_seen}"
    )


def build_execute_request(
    *,
    manipulation_pb2,
    frame_id: str,
    x: float,
    y: float,
    z: float,
    orientation: Quaternion,
    gripper_width: float,
    timeout_s: float,
) -> object:
    req = manipulation_pb2.ExecuteGrasp_Request(
        gripper_width=float(gripper_width),
        timeout_s=float(timeout_s),
    )
    req.target_pose.header.stamp.sec = 0
    req.target_pose.header.stamp.nanosec = 0
    req.target_pose.header.frame_id = frame_id
    req.target_pose.pose.position.x = float(x)
    req.target_pose.pose.position.y = float(y)
    req.target_pose.pose.position.z = float(z)
    req.target_pose.pose.orientation.x = float(orientation.x)
    req.target_pose.pose.orientation.y = float(orientation.y)
    req.target_pose.pose.orientation.z = float(orientation.z)
    req.target_pose.pose.orientation.w = float(orientation.w)
    return req


def main() -> int:
    args = parse_args()
    homography_path = Path(args.homography).expanduser()
    if not homography_path.exists():
        print(f"homography file does not exist: {homography_path}", file=sys.stderr)
        return 2
    homography = np.load(homography_path).astype(np.float64)
    if homography.shape != (3, 3):
        print(f"homography must be 3x3, got {homography.shape}", file=sys.stderr)
        return 2

    rclpy.init()
    node = Handeye2DTestNode(args)
    try:
        node.reset_to_observe_pose()
        orientation = None
        effective_yaw = args.yaw
        if args.orientation_mode in ("current", "vertical-current-yaw"):
            end_pose = node.wait_for_end_pose()
            if args.orientation_mode == "current":
                orientation = end_pose.orientation
            else:
                effective_yaw = quaternion_to_yaw(end_pose.orientation)
        if orientation is None:
            qx, qy, qz, qw = vertical_quaternion(effective_yaw)
            orientation = Quaternion(x=qx, y=qy, z=qz, w=qw)

        node.wait_for_image()
        pixel = pick_pixel(node)
        if pixel is None:
            print("cancelled")
            return 1
        u, v = pixel
        x, y = image_to_robot_xy(homography, u, v)
        print(
            f"clicked pixel=({u}, {v}) -> xy=({x:.4f}, {y:.4f}), "
            f"target {args.frame_id} xyz=({x:.4f}, {y:.4f}, {args.z:.4f}), "
            f"orientation_mode={args.orientation_mode}, yaw={effective_yaw:.4f}"
        )

        (
            grpc,
            channel,
            stub,
            atlas_pb2,
            manipulation_pb2,
            contracts_grpc,
            channel_id,
            endpoint,
        ) = resolve_execute_grasp(args)
        try:
            req = build_execute_request(
                manipulation_pb2=manipulation_pb2,
                frame_id=args.frame_id,
                x=x,
                y=y,
                z=args.z,
                orientation=orientation,
                gripper_width=args.gripper_width,
                timeout_s=args.execute_timeout,
            )
            print(f"calling execute_grasp via {endpoint}")
            exec_channel = grpc.insecure_channel(
                endpoint, options=[("grpc.enable_http_proxy", 0)])
            try:
                exec_stub = contracts_grpc.RobonixServiceManipulationExecuteGraspStub(
                    exec_channel)
                resp = exec_stub.ExecuteGrasp(
                    req, timeout=max(args.execute_timeout + 5.0, 10.0))
                print(
                    "execute_grasp response: "
                    f"success={resp.success} message={resp.message!r} "
                    f"elapsed_s={resp.elapsed_s:.2f}"
                )
                return 0 if bool(resp.success) else 3
            finally:
                try:
                    exec_channel.close()
                except Exception:
                    pass
        finally:
            try:
                stub.DisconnectCapability(
                    atlas_pb2.DisconnectCapabilityRequest(channel_id=channel_id),
                    timeout=2.0,
                )
            except Exception:
                pass
            channel.close()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
