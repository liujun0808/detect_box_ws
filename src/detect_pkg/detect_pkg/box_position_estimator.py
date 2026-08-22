"""Depth ROI processing and fixed-size box center estimation boundary."""

from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Sequence

import numpy as np


class PointCloudExtractionError(RuntimeError):
    """Raised when a detection does not contain enough valid depth data."""


@dataclass(frozen=True)
class CameraPointCloud:
    """Valid depth samples from one YOLO ROI in camera coordinates."""

    points_camera_m: np.ndarray
    pixels_uv: np.ndarray
    depths_m: np.ndarray
    roi_xyxy: tuple[int, int, int, int]

@dataclass(frozen=True)
class PositionEstimate:
    """Estimated geometric center in camera and base coordinates."""

    center_camera_m: tuple[float, float, float]
    center_base_m: tuple[float, float, float]
    confidence: float
    point_count: int


class BoxPositionEstimator:
    """Combines point-cloud filtering, support-plane fitting and center fitting."""

    def __init__(
        self,
        box_size_m: Sequence[float],
        camera_to_base_row_major: Sequence[float],
        parameters: dict[str, Any],
    ) -> None:
        self.box_size_m = tuple(float(value) for value in box_size_m)
        self.camera_to_base_row_major = tuple(
            float(value) for value in camera_to_base_row_major
        )
        self.parameters = parameters

        self._depth_min_m = float(parameters["depth_min_m"])
        self._depth_max_m = float(parameters["depth_max_m"])
        self._bbox_margin_px = int(parameters["depth_bbox_margin_px"])
        self._pixel_stride = int(parameters["pointcloud_pixel_stride"])
        self._min_points = int(parameters["pointcloud_min_points"])
        self._validate_parameters()

    def extract_point_cloud(self, frame: Any, detection: Any) -> CameraPointCloud:
        """Back-project valid aligned depth pixels inside the detected bbox."""
        color_height, color_width = frame.color_bgr.shape[:2]
        depth = np.asarray(frame.depth_u16)
        if depth.ndim != 2 or depth.shape != (color_height, color_width):
            raise PointCloudExtractionError(
                "Aligned depth shape does not match the color image: "
                f"color={(color_height, color_width)}, depth={depth.shape}"
            )
        if not np.isfinite(frame.depth_scale_m) or frame.depth_scale_m <= 0.0:
            raise PointCloudExtractionError(
                f"Invalid depth scale: {frame.depth_scale_m}"
            )

        x_min = max(0, int(detection.x_min) - self._bbox_margin_px)
        y_min = max(0, int(detection.y_min) - self._bbox_margin_px)
        x_max = min(color_width, int(detection.x_max) + self._bbox_margin_px)
        y_max = min(color_height, int(detection.y_max) + self._bbox_margin_px)
        if x_max <= x_min or y_max <= y_min:
            raise PointCloudExtractionError(
                f"Invalid detection ROI: [{x_min}, {y_min}, {x_max}, {y_max}]"
            )

        intrinsics = frame.color_intrinsics
        fx = float(intrinsics.fx)
        fy = float(intrinsics.fy)
        cx = float(intrinsics.ppx)
        cy = float(intrinsics.ppy)
        if not all(np.isfinite(value) and value > 0.0 for value in (fx, fy)):
            raise PointCloudExtractionError(
                f"Invalid aligned camera intrinsics: fx={fx}, fy={fy}"
            )
        if not np.isfinite(cx) or not np.isfinite(cy):
            raise PointCloudExtractionError(
                f"Invalid aligned camera principal point: cx={cx}, cy={cy}"
            )

        sampled_depth = depth[y_min:y_max:self._pixel_stride, x_min:x_max:self._pixel_stride]
        depths_m = sampled_depth.astype(np.float32) * float(frame.depth_scale_m)
        valid = (
            (sampled_depth > 0)
            & np.isfinite(depths_m)
            & (depths_m >= self._depth_min_m)
            & (depths_m <= self._depth_max_m)
        )
        if not np.any(valid):
            raise PointCloudExtractionError(
                "No valid depth samples remained inside the detection ROI"
            )

        v_grid, u_grid = np.mgrid[
            y_min:y_max:self._pixel_stride,
            x_min:x_max:self._pixel_stride,
        ]
        u = u_grid[valid].astype(np.float32)
        v = v_grid[valid].astype(np.float32)
        z = depths_m[valid]
        x = (u - cx) * z / fx
        y = (v - cy) * z / fy
        points_camera_m = np.column_stack((x, y, z)).astype(np.float32)
        pixels_uv = np.column_stack((u, v)).astype(np.float32)

        if points_camera_m.shape[0] < self._min_points:
            raise PointCloudExtractionError(
                "Not enough valid depth samples in the detection ROI: "
                f"{points_camera_m.shape[0]} < {self._min_points}"
            )
        return CameraPointCloud(
            points_camera_m=points_camera_m,
            pixels_uv=pixels_uv,
            depths_m=z.astype(np.float32),
            roi_xyxy=(x_min, y_min, x_max, y_max),
        )

    def estimate(self, frame: Any, detection: Any) -> PositionEstimate:
        del frame, detection
        raise NotImplementedError("Fixed-size box position fitting is not implemented yet")

    def _validate_parameters(self) -> None:
        if not np.isfinite(self._depth_min_m) or not np.isfinite(self._depth_max_m):
            raise ValueError("depth_min_m and depth_max_m must be finite")
        if self._depth_min_m <= 0.0 or self._depth_min_m >= self._depth_max_m:
            raise ValueError("depth_min_m must be positive and less than depth_max_m")
        if self._bbox_margin_px < 0:
            raise ValueError("depth_bbox_margin_px cannot be negative")
        if self._pixel_stride <= 0:
            raise ValueError("pointcloud_pixel_stride must be positive")
        if self._min_points <= 0:
            raise ValueError("pointcloud_min_points must be positive")
