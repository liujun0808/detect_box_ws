"""Synchronized RGB-D acquisition from ROS 2 simulation topics."""

from __future__ import annotations

from dataclasses import dataclass
import threading
import time
from typing import Any

from cv_bridge import CvBridge
from message_filters import ApproximateTimeSynchronizer, Subscriber
import numpy as np
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image


class TopicRgbdCameraError(RuntimeError):
    """Raised when synchronized simulation RGB-D data is unavailable or invalid."""


@dataclass(frozen=True)
class CameraIntrinsics:
    """Pinhole intrinsics in the form expected by the position estimator."""

    fx: float
    fy: float
    ppx: float
    ppy: float


@dataclass(frozen=True)
class AlignedRgbdFrame:
    """One color frame and one metric depth frame aligned to the color image."""

    color_bgr: np.ndarray
    depth_image: np.ndarray
    color_intrinsics: CameraIntrinsics
    depth_scale_m: float
    timestamp_sec: float


class TopicRgbdCamera:
    """Synchronize simulation image topics and provide a fresh frame per request."""

    def __init__(self, node: Any, parameters: dict[str, Any]) -> None:
        self._color_topic = str(parameters["color_topic"]).strip()
        self._camera_info_topic = str(parameters["camera_info_topic"]).strip()
        self._aligned_depth_topic = str(parameters["aligned_depth_topic"]).strip()
        self._sync_queue_size = int(parameters["rgbd_sync_queue_size"])
        self._sync_tolerance_sec = float(parameters["rgbd_sync_tolerance_sec"])
        self._wait_timeout_sec = float(parameters["rgbd_wait_timeout_sec"])
        self._validate_parameters()

        self._bridge = CvBridge()
        self._condition = threading.Condition()
        self._latest_frame: AlignedRgbdFrame | None = None
        self._latest_sequence = 0
        self._last_error = ""

        # Keep topic callbacks independent from the blocking service callback.
        self._subscription_group = MutuallyExclusiveCallbackGroup()
        subscriber_kwargs = {
            "qos_profile": qos_profile_sensor_data,
            "callback_group": self._subscription_group,
        }
        self._color_subscriber = Subscriber(
            node, Image, self._color_topic, **subscriber_kwargs
        )
        self._depth_subscriber = Subscriber(
            node, Image, self._aligned_depth_topic, **subscriber_kwargs
        )
        self._camera_info_subscriber = Subscriber(
            node, CameraInfo, self._camera_info_topic, **subscriber_kwargs
        )
        self._synchronizer = ApproximateTimeSynchronizer(
            [
                self._color_subscriber,
                self._depth_subscriber,
                self._camera_info_subscriber,
            ],
            queue_size=self._sync_queue_size,
            slop=self._sync_tolerance_sec,
            allow_headerless=False,
        )
        self._synchronizer.registerCallback(self._synchronized_callback)

    @property
    def topic_summary(self) -> str:
        return (
            f"color={self._color_topic}, depth={self._aligned_depth_topic}, "
            f"camera_info={self._camera_info_topic}"
        )

    def capture_aligned(self) -> AlignedRgbdFrame:
        """Wait for the first synchronized frame produced after this call."""
        deadline = time.monotonic() + self._wait_timeout_sec
        with self._condition:
            required_sequence = self._latest_sequence + 1
            while self._latest_sequence < required_sequence:
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    details = (
                        f" Last conversion error: {self._last_error}"
                        if self._last_error
                        else ""
                    )
                    raise TopicRgbdCameraError(
                        "Timed out waiting for a new synchronized simulation RGB-D "
                        f"frame after {self._wait_timeout_sec:.3f} s. "
                        f"Topics: {self.topic_summary}.{details}"
                    )
                self._condition.wait(timeout=remaining)
            if self._latest_frame is None:
                raise TopicRgbdCameraError(
                    "RGB-D synchronization advanced without a valid frame"
                )
            return self._latest_frame

    def close(self) -> None:
        """Release ROS subscriptions owned by message_filters."""
        for subscriber in (
            self._color_subscriber,
            self._depth_subscriber,
            self._camera_info_subscriber,
        ):
            try:
                subscriber.unregister()
            except Exception:
                pass

    def _synchronized_callback(
        self,
        color_msg: Image,
        depth_msg: Image,
        camera_info_msg: CameraInfo,
    ) -> None:
        try:
            frame = self._convert_frame(color_msg, depth_msg, camera_info_msg)
        except Exception as error:
            with self._condition:
                self._last_error = str(error)
                self._condition.notify_all()
            return

        with self._condition:
            self._latest_frame = frame
            self._latest_sequence += 1
            self._last_error = ""
            self._condition.notify_all()

    def _convert_frame(
        self,
        color_msg: Image,
        depth_msg: Image,
        camera_info_msg: CameraInfo,
    ) -> AlignedRgbdFrame:
        if color_msg.encoding.lower() != "rgb8":
            raise TopicRgbdCameraError(
                f"Expected color encoding rgb8, got {color_msg.encoding}"
            )
        if depth_msg.encoding.lower() != "32fc1":
            raise TopicRgbdCameraError(
                f"Expected aligned depth encoding 32FC1, got {depth_msg.encoding}"
            )

        color_bgr = np.asarray(
            self._bridge.imgmsg_to_cv2(color_msg, desired_encoding="bgr8"),
            dtype=np.uint8,
        ).copy()
        depth_image = np.asarray(
            self._bridge.imgmsg_to_cv2(depth_msg, desired_encoding="32FC1"),
            dtype=np.float32,
        ).copy()
        if color_bgr.ndim != 3 or color_bgr.shape[2] != 3:
            raise TopicRgbdCameraError(
                f"Unexpected converted color shape: {color_bgr.shape}"
            )
        if depth_image.ndim != 2:
            raise TopicRgbdCameraError(
                f"Unexpected converted depth shape: {depth_image.shape}"
            )
        if color_bgr.shape[:2] != depth_image.shape:
            raise TopicRgbdCameraError(
                "Aligned color and depth image dimensions do not match: "
                f"{color_bgr.shape[:2]} vs {depth_image.shape}"
            )
        if (
            camera_info_msg.width > 0
            and camera_info_msg.height > 0
            and (
                int(camera_info_msg.width) != color_bgr.shape[1]
                or int(camera_info_msg.height) != color_bgr.shape[0]
            )
        ):
            raise TopicRgbdCameraError(
                "CameraInfo dimensions do not match the aligned color image"
            )

        matrix = tuple(float(value) for value in camera_info_msg.k)
        fx, fy, ppx, ppy = matrix[0], matrix[4], matrix[2], matrix[5]
        if not all(np.isfinite(value) for value in (fx, fy, ppx, ppy)):
            raise TopicRgbdCameraError("CameraInfo contains non-finite intrinsics")
        if fx <= 0.0 or fy <= 0.0:
            raise TopicRgbdCameraError(
                f"CameraInfo focal lengths must be positive: fx={fx}, fy={fy}"
            )

        stamp = depth_msg.header.stamp
        timestamp_sec = float(stamp.sec) + float(stamp.nanosec) * 1.0e-9
        return AlignedRgbdFrame(
            color_bgr=color_bgr,
            depth_image=depth_image,
            color_intrinsics=CameraIntrinsics(
                fx=fx,
                fy=fy,
                ppx=ppx,
                ppy=ppy,
            ),
            # 32FC1 depth values are already expressed in metres.
            depth_scale_m=1.0,
            timestamp_sec=timestamp_sec,
        )

    def _validate_parameters(self) -> None:
        topics = (
            self._color_topic,
            self._camera_info_topic,
            self._aligned_depth_topic,
        )
        if not all(topics):
            raise ValueError("Simulation RGB-D topic names cannot be empty")
        if self._sync_queue_size <= 0:
            raise ValueError("rgbd_sync_queue_size must be positive")
        if (
            not np.isfinite(self._sync_tolerance_sec)
            or self._sync_tolerance_sec < 0.0
        ):
            raise ValueError(
                "rgbd_sync_tolerance_sec must be finite and non-negative"
            )
        if not np.isfinite(self._wait_timeout_sec) or self._wait_timeout_sec <= 0.0:
            raise ValueError("rgbd_wait_timeout_sec must be finite and positive")
