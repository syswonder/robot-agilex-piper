#!/usr/bin/env python3
"""2D hand-eye calibration workflow for the rbnx Piper deployment."""

from __future__ import annotations

import argparse
import subprocess
import sys
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
from geometry_msgs.msg import Pose
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from std_srvs.srv import Trigger


DATA_DIR = Path.home() / "lhw" / "rbnx_piper_packages" / "rbnx-boot" / "hand-eye-data"
IMAGE_POINTS_PATH = DATA_DIR / "2d_image_points.npy"
END_POSES_PATH = DATA_DIR / "2d_end_poses.npy"
ROBOT_POINTS_PATH = DATA_DIR / "2d_robot_points.npy"
HOMOGRAPHY_PATH = DATA_DIR / "2d_homography.npy"
REPORT_PATH = DATA_DIR / "2d_homography_report.txt"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument("--mode", choices=("calibrate", "compute", "test"), default="calibrate")
    parser.add_argument("--image-topic", default="/camera/color/image_raw")
    parser.add_argument("--end-pose-topic", default="/arm/end_pose")
    parser.add_argument("--reset-service", default="/moveit_control/reset")
    parser.add_argument("--teach-safe-service", default="/moveit_control/teach_safe")
    parser.add_argument("--enable-service", default="/arm/enable_srv")
    parser.add_argument("--enable-topic", default="/arm/enable_flag")
    parser.add_argument("--image-timeout", type=float, default=10.0)
    parser.add_argument("--service-timeout", type=float, default=90.0)
    parser.add_argument("--post-enable-wait", type=float, default=2.0)
    return parser.parse_args()


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


def pack_end_pose(pose: Pose) -> np.ndarray:
    quat = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
    rz, ry, rx = R.from_quat(quat).as_euler("zyx", degrees=True)
    return np.array(
        [pose.position.x, pose.position.y, pose.position.z, rz, ry, rx],
        dtype=np.float32,
    )


def save_records(image_points: list[tuple[int, int]], end_poses: list[np.ndarray]) -> None:
    DATA_DIR.mkdir(parents=True, exist_ok=True)
    np.save(IMAGE_POINTS_PATH, np.array(image_points, dtype=np.float32))
    np.save(END_POSES_PATH, np.array(end_poses, dtype=np.float32))
    if end_poses:
        robot_points = np.array([[pose[0], pose[1]] for pose in end_poses], dtype=np.float32)
        np.save(ROBOT_POINTS_PATH, robot_points)


def compute_homography() -> np.ndarray:
    image_points = np.load(IMAGE_POINTS_PATH).astype(np.float32)
    end_poses = np.load(END_POSES_PATH).astype(np.float32)
    if len(image_points) != len(end_poses):
        raise RuntimeError(
            f"image/end pose count mismatch: {len(image_points)} vs {len(end_poses)}"
        )
    if len(image_points) < 4 or len(end_poses) < 4:
        raise RuntimeError("至少需要4组点才能计算2D homography")
    robot_points = np.array([[pose[0], pose[1]] for pose in end_poses], dtype=np.float32)
    homography, inlier_mask = cv2.findHomography(
        image_points,
        robot_points,
        method=cv2.RANSAC,
        ransacReprojThreshold=0.005,
        maxIters=10000,
        confidence=0.999,
    )
    if homography is None:
        raise RuntimeError("cv2.findHomography failed")
    projected = np.array([image_to_robot_xy(homography, point) for point in image_points], dtype=np.float32)
    errors = np.linalg.norm(projected - robot_points, axis=1)
    DATA_DIR.mkdir(parents=True, exist_ok=True)
    np.save(HOMOGRAPHY_PATH, homography.astype(np.float32))
    np.save(ROBOT_POINTS_PATH, robot_points)
    mask_text = "None" if inlier_mask is None else inlier_mask.reshape(-1).astype(int).tolist()
    report = (
        f"homography:\n{homography}\n\n"
        f"inlier_mask: {mask_text}\n"
        f"errors_m: {errors.tolist()}\n"
        f"mean_error_m: {float(errors.mean())}\n"
        f"max_error_m: {float(errors.max())}\n"
    )
    REPORT_PATH.write_text(report, encoding="utf-8")
    print(report)
    print(f"saved homography: {HOMOGRAPHY_PATH}")
    return homography


def image_to_robot_xy(homography: np.ndarray, point: np.ndarray) -> tuple[float, float]:
    u, v = float(point[0]), float(point[1])
    projected = homography @ np.array([u, v, 1.0], dtype=np.float64)
    if abs(float(projected[2])) < 1e-9:
        raise ValueError("homography projection has near-zero scale")
    projected = projected / projected[2]
    return float(projected[0]), float(projected[1])


def wait_for_disable_confirmation(node: "CalibrateNode") -> bool:
    print("请确认机械臂已经停在安全失能姿态。确认后按 d 失能；按 Esc 取消本点。")
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.05)
        key = cv2.waitKey(50) & 0xFF
        if key in (27, ord("q")):
            print("已取消失能，仍保持使能状态。")
            return False
        if key == ord("d"):
            return True


class CalibrateNode(Node):
    def __init__(self, args: argparse.Namespace):
        super().__init__("handeye_2d_calibrate")
        self.args = args
        self.latest_image: Image | None = None
        self.latest_end_pose: Pose | None = None
        self.latest_end_pose_seq = 0
        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.BEST_EFFORT
        qos.durability = DurabilityPolicy.VOLATILE
        self.create_subscription(Image, args.image_topic, self._image_cb, qos)
        self.create_subscription(Pose, args.end_pose_topic, self._pose_cb, 1)
        self.reset_client = self.create_client(Trigger, args.reset_service)
        self.teach_safe_client = self.create_client(Trigger, args.teach_safe_service)
        try:
            from piper_msgs.srv import Enable
        except ModuleNotFoundError as e:
            raise RuntimeError(
                "piper_msgs is not importable. Source the piper_ctl overlay first: "
                "source rbnx-boot/cache/piper_ctl_rbnx/rbnx-build/ws/install/setup.bash"
            ) from e
        self.Enable = Enable
        self.enable_client = self.create_client(Enable, args.enable_service)
        self.enable_pub = self.create_publisher(Bool, args.enable_topic, 10)

    def _image_cb(self, msg: Image) -> None:
        self.latest_image = msg

    def _pose_cb(self, msg: Pose) -> None:
        self.latest_end_pose = msg
        self.latest_end_pose_seq += 1

    def call_trigger(self, client, name: str) -> None:
        self.get_logger().info(f"waiting for {name}")
        if not client.wait_for_service(timeout_sec=self.args.service_timeout):
            raise TimeoutError(f"service unavailable: {name}")
        future = client.call_async(Trigger.Request())
        self._spin_until_future(future, self.args.service_timeout, name)
        resp = future.result()
        if resp is None or not resp.success:
            msg = "" if resp is None else resp.message
            raise RuntimeError(f"{name} failed: {msg}")
        self.get_logger().info(f"{name} ok: {resp.message}")

    def set_enabled(self, enabled: bool) -> None:
        name = f"{self.args.enable_service} -> {enabled}"
        self.get_logger().info(f"waiting for {name}")
        if not self.enable_client.wait_for_service(timeout_sec=self.args.service_timeout):
            raise TimeoutError(f"service unavailable: {self.args.enable_service}")
        req = self.Enable.Request()
        req.enable_request = bool(enabled)
        future = self.enable_client.call_async(req)
        self._spin_until_future(future, self.args.service_timeout, name)
        resp = future.result()
        if resp is None or not resp.enable_response:
            self.get_logger().warning(
                f"{name} service returned failure; falling back to {self.args.enable_topic}"
            )
            msg = Bool()
            msg.data = bool(enabled)
            for _ in range(5):
                self.enable_pub.publish(msg)
                rclpy.spin_once(self, timeout_sec=0.1)
                time.sleep(0.1)
            return
        self.get_logger().info(f"{name} ok")

    def _spin_until_future(self, future, timeout_s: float, name: str) -> None:
        deadline = time.monotonic() + timeout_s
        while rclpy.ok() and not future.done() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        if not future.done():
            raise TimeoutError(f"{name} timed out")

    def wait_for_image(self) -> Image:
        self.get_logger().info(f"waiting for image on {self.args.image_topic}")
        deadline = time.monotonic() + self.args.image_timeout
        while rclpy.ok() and self.latest_image is None and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        if self.latest_image is None:
            raise TimeoutError(f"no image on {self.args.image_topic}")
        return self.latest_image

    def get_latest_pose(self) -> Pose:
        deadline = time.monotonic() + self.args.image_timeout
        while rclpy.ok() and self.latest_end_pose is None and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
        if self.latest_end_pose is None:
            raise TimeoutError(f"no pose on {self.args.end_pose_topic}")
        return self.latest_end_pose


def run_calibrate(args: argparse.Namespace) -> None:
    DATA_DIR.mkdir(parents=True, exist_ok=True)
    image_points: list[tuple[int, int]] = []
    end_poses: list[np.ndarray] = []
    if IMAGE_POINTS_PATH.exists() and END_POSES_PATH.exists():
        image_points = [tuple(map(int, row)) for row in np.load(IMAGE_POINTS_PATH).tolist()]
        end_poses = [np.array(row, dtype=np.float32) for row in np.load(END_POSES_PATH)]
        print(f"loaded existing records: {len(image_points)}")

    selected_point: tuple[int, int] | None = None
    state = "observe"
    window_name = "handeye_2d_calibrate"

    def on_mouse(event, x, y, _flags, _param):
        nonlocal selected_point
        if event == cv2.EVENT_LBUTTONDOWN:
            selected_point = (int(x), int(y))
            print(f"选择图片点: {selected_point}")

    rclpy.init()
    node = CalibrateNode(args)
    cv2.namedWindow(window_name, cv2.WINDOW_AUTOSIZE)
    cv2.setMouseCallback(window_name, on_mouse)
    try:
        print("正在使能机械臂并回到固定观察位姿...")
        node.set_enabled(True)
        time.sleep(max(0.0, args.post_enable_wait))
        node.call_trigger(node.reset_client, args.reset_service)
        image_msg = node.wait_for_image()
        frozen_image = ros_image_to_bgr(image_msg)
        print(f"图像分辨率: {frozen_image.shape[1]}x{frozen_image.shape[0]}")
        print("点击图像点后按 Enter/空格确认；Esc 退出。")
        try:
            while rclpy.ok():
                rclpy.spin_once(node, timeout_sec=0.0)
                if state == "observe" and node.latest_image is not None:
                    frozen_image = ros_image_to_bgr(node.latest_image)
                frame = frozen_image.copy()
                if selected_point is not None:
                    cv2.circle(frame, selected_point, 6, (0, 0, 255), 2)
                cv2.putText(
                    frame,
                    f"pairs: {len(image_points)}  state: {state}",
                    (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.8,
                    (0, 255, 0),
                    2,
                )
                cv2.imshow(window_name, frame)
                key = cv2.waitKey(30) & 0xFF
                if key == 27:
                    break
                if state == "observe" and key in (13, 32):
                    if selected_point is None:
                        print("请先点击图片点。")
                        continue
                    print("正在移动到安全失能位姿...")
                    node.call_trigger(node.teach_safe_client, args.teach_safe_service)
                    if not wait_for_disable_confirmation(node):
                        state = "observe"
                        continue
                    print("正在失能，请手动拖动机械臂到目标点，完成后按空格记录。")
                    node.set_enabled(False)
                    state = "manual"
                    continue
                if state == "manual" and key == 32:
                    pose = node.get_latest_pose()
                    end_pose = pack_end_pose(pose)
                    image_points.append(selected_point)
                    end_poses.append(end_pose)
                    save_records(image_points, end_poses)
                    print("记录图片点:", selected_point)
                    print("记录末端位姿 [x, y, z, RZ, RY, RX]:", end_pose.tolist())
                    print(f"已保存 {len(image_points)} 组。正在重新使能并回观察位姿...")
                    selected_point = None
                    node.set_enabled(True)
                    time.sleep(max(0.0, args.post_enable_wait))
                    node.call_trigger(node.reset_client, args.reset_service)
                    image_msg = node.wait_for_image()
                    frozen_image = ros_image_to_bgr(image_msg)
                    print(f"图像分辨率: {frozen_image.shape[1]}x{frozen_image.shape[0]}")
                    state = "observe"
                    print("继续点击下一点；Esc 退出。")
        except KeyboardInterrupt:
            print("\n收到 Ctrl-C，保存已有数据并计算 homography。")
    finally:
        save_records(image_points, end_poses)
        cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    if len(image_points) >= 4:
        compute_homography()
    else:
        print(f"当前只有 {len(image_points)} 组，至少4组后再计算 homography。")


def run_test_passthrough() -> int:
    script = Path(__file__).with_name("test_handeye_2d.py")
    return subprocess.call([sys.executable, str(script)])


def main() -> int:
    args = parse_args()
    if args.mode == "compute":
        compute_homography()
        return 0
    if args.mode == "test":
        return run_test_passthrough()
    run_calibrate(args)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
