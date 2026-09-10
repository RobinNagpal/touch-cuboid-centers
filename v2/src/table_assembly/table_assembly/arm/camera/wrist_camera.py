"""The camera bolted to the wrist.

The camera moves with the arm, so a frame is only useful together with the
pose the arm was holding when it was taken. This class hands both back at
once: the images, the intrinsics, and the 4x4 transform that puts a pixel in
the world frame.

Set ``TABLE_ASSEMBLY_VIEWS`` to a folder and every frame taken is also saved
there, with its pose, for looking at afterwards.
"""

from __future__ import annotations

import os
import threading
import time
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import Buffer, TransformListener

from ...messages import transform_to_matrix
from ...perception.pixels import Intrinsics
from ..dimensions import WORLD_FRAME


@dataclass(frozen=True)
class View:
    """One RGB-D frame and where the camera was when it was taken."""

    rgb: np.ndarray
    depth: np.ndarray
    intrinsics: Intrinsics
    camera_to_world: np.ndarray


class CaptureTimeout(RuntimeError):
    """Raised when the camera did not deliver a frame in time."""


class WristCamera:
    def __init__(
        self,
        node: Node,
        *,
        image_topic: str = "/wrist_camera/image",
        depth_topic: str = "/wrist_camera/depth_image",
        info_topic: str = "/wrist_camera/camera_info",
        optical_frame: str = "wrist_camera_optical_frame",
    ) -> None:
        self._node = node
        self._optical_frame = optical_frame
        self._bridge = CvBridge()
        self._lock = threading.Lock()
        self._rgb: Image | None = None
        self._depth: Image | None = None
        self._info: CameraInfo | None = None
        self._counts = {"rgb": 0, "depth": 0, "info": 0}
        self._save_to = (
            Path(os.environ["TABLE_ASSEMBLY_VIEWS"]) if os.environ.get("TABLE_ASSEMBLY_VIEWS") else None
        )
        self._saved = 0

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, node)

        # Its own callback group, so that a multi-threaded executor can deliver
        # camera frames on one thread while another is busy with the simulated
        # clock, which ticks far more often than the camera does.
        group = MutuallyExclusiveCallbackGroup()
        sensor = qos_profile_sensor_data
        node.create_subscription(Image, image_topic, self._on_rgb, sensor, callback_group=group)
        node.create_subscription(Image, depth_topic, self._on_depth, sensor, callback_group=group)
        node.create_subscription(CameraInfo, info_topic, self._on_info, sensor, callback_group=group)

    def _on_rgb(self, msg: Image) -> None:
        with self._lock:
            self._rgb = msg
            self._counts["rgb"] += 1

    def _on_depth(self, msg: Image) -> None:
        with self._lock:
            self._depth = msg
            self._counts["depth"] += 1

    def _on_info(self, msg: CameraInfo) -> None:
        with self._lock:
            self._info = msg
            self._counts["info"] += 1

    def wait_until_ready(self, timeout: float = 120.0) -> None:
        """Block until the first frames and the intrinsics have arrived.

        The camera only starts publishing once Gazebo has the sensor running,
        which is later than the rest of the cell comes up.
        """
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            with self._lock:
                ready = self._rgb is not None and self._depth is not None and self._info is not None
            if ready:
                return
            time.sleep(0.05)

        raise CaptureTimeout(f"no camera frames within {timeout:.0f}s (messages so far: {self._counts})")

    def capture(self, timeout: float = 10.0) -> View:
        """Take a fresh frame, and the pose the camera was at when it was taken.

        Two things have to line up for a pixel to land in the right place,
        and neither can be taken for granted.

        The frame has to be taken after the arm stopped. A frame that was
        rendered a moment before, while the arm was still arriving, can reach
        this node after the arm has stopped. So only frames stamped later than
        the moment this was called are accepted.

        The pose has to be the one the camera was at when the frame was taken,
        not the latest one known. Those differ by however far the arm's joint
        states lag behind the images, and a pose a few centimetres off puts
        every point of the picture a few centimetres off. So the pose is
        looked up at the frame's own timestamp.
        """
        asked = self._node.get_clock().now()
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            with self._lock:
                rgb, depth, info = self._rgb, self._depth, self._info
            if rgb is not None and depth is not None and info is not None and _taken_after(asked, rgb, depth):
                view = self._build_view(rgb, depth, info)
                if self._save_to is not None:
                    self._save(view)
                return view
            time.sleep(0.02)

        raise CaptureTimeout(f"no RGB-D frame within {timeout:.0f}s (messages so far: {self._counts})")

    def _build_view(self, rgb: Image, depth: Image, info: CameraInfo) -> View:
        transform = self._tf_buffer.lookup_transform(
            WORLD_FRAME,
            self._optical_frame,
            rclpy.time.Time.from_msg(depth.header.stamp),
            timeout=Duration(seconds=2.0),
        )
        return View(
            rgb=self._bridge.imgmsg_to_cv2(rgb, desired_encoding="rgb8"),
            depth=np.asarray(self._bridge.imgmsg_to_cv2(depth, desired_encoding="32FC1"), dtype=float),
            intrinsics=Intrinsics.from_camera_info(info),
            camera_to_world=transform_to_matrix(transform.transform),
        )

    def _save(self, view: View) -> None:
        self._save_to.mkdir(parents=True, exist_ok=True)
        k = view.intrinsics
        np.savez_compressed(
            self._save_to / f"view_{self._saved:03d}.npz",
            rgb=view.rgb,
            depth=view.depth,
            intrinsics=np.array([k.fx, k.fy, k.cx, k.cy]),
            camera_to_world=view.camera_to_world,
        )
        self._saved += 1


def _taken_after(moment, *images: Image) -> bool:
    return all(rclpy.time.Time.from_msg(image.header.stamp) >= moment for image in images)
