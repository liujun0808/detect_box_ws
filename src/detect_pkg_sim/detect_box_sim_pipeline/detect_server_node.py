"""ROS 2 service entry point for request-triggered box position estimation."""

from __future__ import annotations

import math
import threading
import time
from typing import Any, Sequence

import rclpy
from geometry_msgs.msg import Pose
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from upper_limb_interface.srv import DetectAprilTag
from visualization_msgs.msg import Marker

from detect_box_sim_pipeline.box_position_estimator import (
    BoxPositionEstimator,
    PointCloudExtractionError,
    PointCloudProcessingError,
    PositionEstimationError,
)
from detect_box_sim_pipeline.debug_snapshot import DebugSnapshotWriter
from detect_box_sim_pipeline.topic_rgbd_camera import (
    TopicRgbdCamera,
    TopicRgbdCameraError,
)
from detect_box_sim_pipeline.yolo_world_detector import (
    YoloWorldDetector,
    YoloWorldDetectorError,
)


class DetectServerNode(Node):
    """Preserve /detect while acquiring aligned RGB-D from simulation topics."""

    def __init__(self) -> None:
        super().__init__(
            "detect_server_node",
            automatically_declare_parameters_from_overrides=True,
        )
        self._request_lock = threading.Lock()
        self._service_name = str(self._parameter("service_name", "/detect"))
        self._pose_frame_id = str(self._parameter("box_pose_frame_id", "base_link"))
        pose_topic = str(self._parameter("box_pose_topic", "box_pose"))

        self._fallback_pose = self._pose_from_values(
            self._parameter("fallback_pose", [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0])
        )
        camera_to_base = self._float_array(
            "camera_to_base_row_major",
            self._parameter("camera_to_base_row_major", self._identity_matrix()),
            16,
        )
        box_size = (
            float(self._parameter("box_size_x_m", 0.295)),
            float(self._parameter("box_size_y_m", 0.395)),
            float(self._parameter("box_size_z_m", 0.225)),
        )
        class_prompts = tuple(
            str(value) for value in self._parameter("yolo_class_prompts", ["plastic crate"])
        )

        self._camera = TopicRgbdCamera(self, self._camera_parameters())
        self._detector = YoloWorldDetector(
            str(self._parameter("yolo_model_path", "")),
            class_prompts,
            self._yolo_parameters(),
        )
        self.get_logger().info("Loading YOLO-World and CLIP model at startup...")
        try:
            self._detector.initialize(
                warmup=bool(self._parameter("yolo_startup_warmup", True))
            )
        except YoloWorldDetectorError as error:
            self.get_logger().error(f"Startup model initialization failed: {error}")
            raise
        self.get_logger().info("YOLO-World startup initialization complete")
        self._estimator = BoxPositionEstimator(
            box_size,
            camera_to_base,
            self._estimator_parameters(),
        )
        self._debug_snapshot_writer = DebugSnapshotWriter(self._debug_parameters())

        self._marker_publisher = self.create_publisher(Marker, pose_topic, 1)
        self._service = self.create_service(
            DetectAprilTag,
            self._service_name,
            self._handle_detect,
        )
        self.get_logger().info(
            f"Simulation box-position service ready: service={self._service_name}, "
            f"frame={self._pose_frame_id}, prompts={len(class_prompts)}, "
            f"{self._camera.topic_summary}"
        )

    def _parameter(self, name: str, default: Any) -> Any:
        if not self.has_parameter(name):
            self.declare_parameter(name, default)
        return self.get_parameter(name).value

    def _camera_parameters(self) -> dict[str, Any]:
        defaults = {
            "color_topic": "/camera/color/image_raw",
            "camera_info_topic": "/camera/color/camera_info",
            "aligned_depth_topic": "/camera/aligned_depth_to_color/image_raw",
            "rgbd_sync_queue_size": 20,
            "rgbd_sync_tolerance_sec": 0.03,
            "rgbd_wait_timeout_sec": 1.0,
        }
        return {
            name: self._parameter(name, default)
            for name, default in defaults.items()
        }

    def _estimator_parameters(self) -> dict[str, Any]:
        prefixes = (
            "depth_",
            "pointcloud_",
            "inner_wall_",
            "top_edge_",
            "box_coordinate_",
            "optimizer_",
            "output_base_",
        )
        return {
            parameter.name: parameter.value
            for parameter in self._parameters.values()
            if parameter.name.startswith(prefixes)
        }

    def _yolo_parameters(self) -> dict[str, Any]:
        defaults = {
            "yolo_device": "auto",
            "yolo_image_size": 640,
            "yolo_confidence_threshold": 0.25,
            "yolo_iou_threshold": 0.45,
            "yolo_agnostic_nms": True,
            "yolo_max_detections": 10,
            "yolo_min_bbox_width_px": 20,
            "yolo_min_bbox_height_px": 20,
            "yolo_startup_warmup": True,
        }
        return {
            name: self._parameter(name, default)
            for name, default in defaults.items()
        }

    def _debug_parameters(self) -> dict[str, Any]:
        defaults = {
            "debug_enabled": True,
            "debug_output_dir": "debug_box_position",
            "debug_max_snapshots": 10,
        }
        return {
            name: self._parameter(name, default)
            for name, default in defaults.items()
        }

    def _handle_detect(
        self,
        request: DetectAprilTag.Request,
        response: DetectAprilTag.Response,
    ) -> DetectAprilTag.Response:
        response.success = False
        response.box_pose = self._fallback_pose
        if not request.capture_once:
            response.message = "capture_once=false, detection was not triggered"
            self.get_logger().info(
                f"Detection response: success={response.success}, "
                f"message: {response.message}"
            )
            return response
        if not self._request_lock.acquire(blocking=False):
            response.message = "A detection request is already running"
            self.get_logger().warning(
                f"Detection response: success={response.success}, "
                f"message: {response.message}"
            )
            return response
        frame = None
        detection = None
        point_cloud = None
        processing = None
        estimate = None
        timings: dict[str, float] = {}
        request_start = time.perf_counter()
        try:
            stage_start = time.perf_counter()
            frame = self._camera.capture_aligned()
            timings["capture"] = (time.perf_counter() - stage_start) * 1000.0

            stage_start = time.perf_counter()
            detection = self._detector.detect_one(frame.color_bgr)
            timings["yolo"] = (time.perf_counter() - stage_start) * 1000.0
            if self._detector.last_inference_device is not None:
                self.get_logger().info(
                    "YOLO actual inference device: "
                    f"{self._detector.last_inference_device}"
                )
            if detection is None:
                response.message = "RGB-D capture succeeded, but YOLO found no crate"
                return response

            stage_start = time.perf_counter()
            point_cloud = self._estimator.extract_point_cloud(frame, detection)
            timings["pointcloud_extract"] = (
                time.perf_counter() - stage_start
            ) * 1000.0

            stage_start = time.perf_counter()
            processing = self._estimator.process_point_cloud(point_cloud)
            timings["pointcloud_process"] = (
                time.perf_counter() - stage_start
            ) * 1000.0

            stage_start = time.perf_counter()
            estimate = self._estimator.estimate_from_processing(
                frame,
                detection,
                processing,
            )
            timings["optimizer"] = (time.perf_counter() - stage_start) * 1000.0
            response.success = True
            response.box_pose = self._pose_from_center(estimate.center_base_m)
            response.message = (
                "Box position estimation succeeded: "
                f"class={detection.class_name}, "
                f"confidence={detection.confidence:.3f}, "
                f"bbox=[{detection.x_min},{detection.y_min},"
                f"{detection.x_max},{detection.y_max}], "
                f"center_base={estimate.center_base_m}, "
                f"center_camera={estimate.center_camera_m}, "
                f"points={estimate.point_count}, "
                f"confidence={estimate.confidence:.3f}, "
                f"surface_rmse_m={estimate.surface_rmse_m:.4f}, "
                f"surface_inlier_ratio={estimate.surface_inlier_ratio:.3f}, "
                "support_plane=disabled, "
                f"inner_wall={'valid' if estimate.inner_wall_valid else 'not found'}"
            )
            self.get_logger().info(
                "Box center in camera frame: "
                f"x={estimate.center_camera_m[0]:.4f} m, "
                f"y={estimate.center_camera_m[1]:.4f} m, "
                f"z={estimate.center_camera_m[2]:.4f} m"
            )
            self.get_logger().info(
                "Box center in base_link frame: "
                f"x={estimate.center_base_m[0]:.4f} m, "
                f"y={estimate.center_base_m[1]:.4f} m, "
                f"z={estimate.center_base_m[2]:.4f} m"
            )
            return response
        except (
            TopicRgbdCameraError,
            YoloWorldDetectorError,
            PointCloudExtractionError,
            PointCloudProcessingError,
            PositionEstimationError,
        ) as error:
            response.message = str(error)
            self.get_logger().error(response.message)
            return response
        finally:
            timings["pipeline"] = (time.perf_counter() - request_start) * 1000.0
            debug_start = time.perf_counter()
            if frame is not None:
                try:
                    snapshot_dir = self._debug_snapshot_writer.save(
                        frame,
                        detection,
                        processing,
                        estimate.center_camera_m if estimate is not None else None,
                    )
                    if snapshot_dir is not None:
                        self.get_logger().info(
                            f"Debug snapshot saved: {snapshot_dir}"
                        )
                except Exception as error:  # pragma: no cover - filesystem/runtime issue
                    self.get_logger().warning(
                        f"Failed to save debug snapshot; detection result is preserved: {error}"
                    )
            timings["debug_snapshot"] = (
                time.perf_counter() - debug_start
            ) * 1000.0
            timings["total"] = (time.perf_counter() - request_start) * 1000.0
            timing_text = ", ".join(
                f"{name}={duration:.1f} ms" for name, duration in timings.items()
            )
            self.get_logger().info(f"Detection timing: {timing_text}")
            self.get_logger().info(
                f"Detection response: success={response.success}, "
                f"message: {response.message}"
            )
            self._request_lock.release()

    @staticmethod
    def _identity_matrix() -> list[float]:
        return [
            1.0, 0.0, 0.0, 0.0,
            0.0, 1.0, 0.0, 0.0,
            0.0, 0.0, 1.0, 0.0,
            0.0, 0.0, 0.0, 1.0,
        ]

    @staticmethod
    def _float_array(name: str, values: Sequence[Any], expected_size: int) -> tuple[float, ...]:
        result = tuple(float(value) for value in values)
        if len(result) != expected_size or not all(math.isfinite(value) for value in result):
            raise ValueError(f"{name} must contain {expected_size} finite values")
        return result

    @classmethod
    def _pose_from_values(cls, values: Sequence[Any]) -> Pose:
        data = cls._float_array("fallback_pose", values, 7)
        quaternion_norm = math.sqrt(sum(value * value for value in data[3:]))
        if quaternion_norm <= 1.0e-9:
            raise ValueError("fallback_pose quaternion norm must be greater than zero")
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = data[:3]
        pose.orientation.x = data[3] / quaternion_norm
        pose.orientation.y = data[4] / quaternion_norm
        pose.orientation.z = data[5] / quaternion_norm
        pose.orientation.w = data[6] / quaternion_norm
        return pose

    @staticmethod
    def _pose_from_center(center_base: Sequence[float]) -> Pose:
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = (
            float(value) for value in center_base
        )
        pose.orientation.w = 1.0
        return pose

    def destroy_node(self) -> bool:
        self._camera.close()
        return super().destroy_node()


def main(args: Sequence[str] | None = None) -> None:
    rclpy.init(args=args)
    node = DetectServerNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
