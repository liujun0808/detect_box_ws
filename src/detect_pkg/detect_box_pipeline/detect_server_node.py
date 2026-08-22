"""ROS 2 service entry point for request-triggered box position estimation."""

from __future__ import annotations

import math
import threading
from typing import Any, Sequence

import rclpy
from geometry_msgs.msg import Pose
from rclpy.node import Node
from upper_limb_interface.srv import DetectAprilTag
from visualization_msgs.msg import Marker

from detect_box_pipeline.box_position_estimator import (
    BoxPositionEstimator,
    PointCloudExtractionError,
    PointCloudProcessingError,
    PositionEstimationError,
)
from detect_box_pipeline.debug_snapshot import DebugSnapshotWriter
from detect_box_pipeline.realsense_camera import RealSenseCamera, RealSenseCameraError
from detect_box_pipeline.yolo_world_detector import (
    YoloWorldDetector,
    YoloWorldDetectorError,
)


class DetectServerNode(Node):
    """Preserves the original service while hosting the new RGB-D pipeline."""

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

        self._camera = RealSenseCamera(self._camera_parameters())
        self._detector = YoloWorldDetector(
            str(self._parameter("yolo_model_path", "")),
            class_prompts,
            self._yolo_parameters(),
        )
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
            f"New box-position service scaffold ready: service={self._service_name}, "
            f"frame={self._pose_frame_id}, prompts={len(class_prompts)}"
        )

    def _parameter(self, name: str, default: Any) -> Any:
        if not self.has_parameter(name):
            self.declare_parameter(name, default)
        return self.get_parameter(name).value

    def _camera_parameters(self) -> dict[str, Any]:
        defaults = {
            "camera_serial_no": "",
            "camera_width": 640,
            "camera_height": 480,
            "camera_fps": 30,
            "camera_warmup_frames": 15,
            "camera_startup_timeout_ms": 5000,
            "camera_frame_timeout_ms": 1000,
            "camera_keep_running": True,
            "camera_reset_on_start_failure": True,
            "camera_reset_reconnect_timeout_ms": 10000,
            "camera_reset_settle_ms": 2000,
        }
        return {
            name: self._parameter(name, default)
            for name, default in defaults.items()
        }

    def _estimator_parameters(self) -> dict[str, Any]:
        prefixes = (
            "depth_",
            "pointcloud_",
            "support_plane_",
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
            return response
        if not self._request_lock.acquire(blocking=False):
            response.message = "A detection request is already running"
            return response
        frame = None
        detection = None
        point_cloud = None
        processing = None
        estimate = None
        failure_message: str | None = None
        try:
            frame = self._camera.capture_aligned()
            detection = self._detector.detect_one(frame.color_bgr)
            if detection is None:
                response.message = "RGB-D capture succeeded, but YOLO found no crate"
                failure_message = response.message
                return response
            point_cloud = self._estimator.extract_point_cloud(frame, detection)
            processing = self._estimator.process_point_cloud(point_cloud)
            estimate = self._estimator.estimate_from_processing(
                frame,
                detection,
                processing,
            )
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
                f"support_plane={'valid' if estimate.support_plane_valid else 'not found'}"
            )
            return response
        except (
            RealSenseCameraError,
            YoloWorldDetectorError,
            PointCloudExtractionError,
            PointCloudProcessingError,
            PositionEstimationError,
        ) as error:
            response.message = str(error)
            failure_message = response.message
            self.get_logger().error(response.message)
            return response
        finally:
            if frame is not None:
                try:
                    snapshot_dir = self._debug_snapshot_writer.save(
                        frame,
                        detection,
                        point_cloud,
                        processing,
                        estimate,
                        failure_message,
                    )
                    if snapshot_dir is not None:
                        self.get_logger().info(
                            f"Debug snapshot saved: {snapshot_dir}"
                        )
                except Exception as error:  # pragma: no cover - filesystem/runtime issue
                    self.get_logger().warning(
                        f"Failed to save debug snapshot; detection result is preserved: {error}"
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
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
