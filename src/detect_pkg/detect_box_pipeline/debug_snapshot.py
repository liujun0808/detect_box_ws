"""Per-request RGB-D, bounding-box and point-cloud debug snapshots."""

from __future__ import annotations

import json
import shutil
from datetime import datetime
from pathlib import Path
from typing import Any

import cv2
import numpy as np


class DebugSnapshotWriter:
    """Persist one estimation attempt and retain only recent snapshots."""

    def __init__(self, parameters: dict[str, Any]) -> None:
        self._enabled = bool(parameters.get("debug_enabled", False))
        output_dir = str(parameters.get("debug_output_dir", "")).strip()
        self._root_dir = Path(output_dir).expanduser() if output_dir else None
        self._max_snapshots = max(1, int(parameters.get("debug_max_snapshots", 10)))

    def save(
        self,
        frame: Any,
        detection: Any | None = None,
        point_cloud: Any | None = None,
        processing: Any | None = None,
        estimate: Any | None = None,
        error: str | None = None,
    ) -> Path | None:
        """Save available data for one request and return its directory."""
        if not self._enabled or self._root_dir is None:
            return None

        self._root_dir.mkdir(parents=True, exist_ok=True)
        snapshot_dir = self._new_snapshot_dir()
        snapshot_dir.mkdir()
        files: list[str] = []

        color = np.asarray(frame.color_bgr)
        self._write_image(snapshot_dir / "color.png", color)
        files.append("color.png")
        if detection is not None:
            annotated = color.copy()
            self._draw_detection(annotated, detection)
            self._write_image(snapshot_dir / "color_with_yolo_bbox.png", annotated)
            files.append("color_with_yolo_bbox.png")

        depth_u16 = np.asarray(frame.depth_u16)
        self._write_image(snapshot_dir / "depth_u16.png", depth_u16)
        files.append("depth_u16.png")
        depth_visual = self._depth_visualization(depth_u16)
        if depth_visual is not None:
            self._write_image(snapshot_dir / "depth_visual.png", depth_visual)
            files.append("depth_visual.png")

        if point_cloud is not None:
            self._write_ply(
                snapshot_dir / "yolo_roi_cloud.ply",
                point_cloud.points_camera_m,
            )
            files.append("yolo_roi_cloud.ply")
        if processing is not None:
            self._write_ply(
                snapshot_dir / "final_candidate_cloud.ply",
                processing.selected_cloud.points_camera_m,
            )
            files.append("final_candidate_cloud.ply")

        metadata = self._metadata(
            frame,
            detection,
            point_cloud,
            processing,
            estimate,
            error,
            files,
        )
        (snapshot_dir / "metadata.json").write_text(
            json.dumps(metadata, ensure_ascii=False, indent=2),
            encoding="utf-8",
        )
        self._remove_old_snapshots()
        return snapshot_dir

    def _new_snapshot_dir(self) -> Path:
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        candidate = self._root_dir / timestamp
        suffix = 1
        while candidate.exists():
            candidate = self._root_dir / f"{timestamp}_{suffix:02d}"
            suffix += 1
        return candidate

    @staticmethod
    def _write_image(path: Path, image: np.ndarray) -> None:
        if image.size == 0 or not cv2.imwrite(str(path), image):
            raise OSError(f"Failed to write debug image: {path}")

    @staticmethod
    def _draw_detection(image: np.ndarray, detection: Any) -> None:
        height, width = image.shape[:2]
        x_min = max(0, min(width - 1, int(detection.x_min)))
        y_min = max(0, min(height - 1, int(detection.y_min)))
        x_max = max(0, min(width - 1, int(detection.x_max)))
        y_max = max(0, min(height - 1, int(detection.y_max)))
        cv2.rectangle(image, (x_min, y_min), (x_max, y_max), (0, 255, 0), 2)
        label = f"{detection.class_name} {float(detection.confidence):.3f}"
        cv2.putText(
            image,
            label,
            (x_min, max(18, y_min - 6)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.5,
            (0, 255, 0),
            1,
            cv2.LINE_AA,
        )

    @staticmethod
    def _depth_visualization(depth_u16: np.ndarray) -> np.ndarray | None:
        valid = depth_u16[depth_u16 > 0]
        if valid.size == 0:
            return None
        low, high = np.percentile(valid, (2.0, 98.0))
        if high <= low:
            high = low + 1.0
        normalized = np.clip(
            (depth_u16.astype(np.float32) - low) / (high - low), 0.0, 1.0
        )
        image = (normalized * 255.0).astype(np.uint8)
        image[depth_u16 <= 0] = 0
        return cv2.applyColorMap(image, cv2.COLORMAP_JET)

    @staticmethod
    def _write_ply(path: Path, points: Any) -> None:
        array = np.asarray(points, dtype=np.float64)
        if array.size == 0:
            array = np.empty((0, 3), dtype=np.float64)
        if array.ndim != 2 or array.shape[1] != 3:
            raise ValueError(f"Point cloud must have shape (N, 3), got {array.shape}")
        finite = np.all(np.isfinite(array), axis=1)
        array = array[finite]
        with path.open("w", encoding="ascii") as stream:
            stream.write("ply\nformat ascii 1.0\n")
            stream.write(f"element vertex {array.shape[0]}\n")
            stream.write("property float x\nproperty float y\nproperty float z\n")
            stream.write("end_header\n")
            for x, y, z in array:
                stream.write(f"{x:.7f} {y:.7f} {z:.7f}\n")

    @staticmethod
    def _metadata(
        frame: Any,
        detection: Any | None,
        point_cloud: Any | None,
        processing: Any | None,
        estimate: Any | None,
        error: str | None,
        files: list[str],
    ) -> dict[str, Any]:
        data: dict[str, Any] = {
            "saved_at": datetime.now().isoformat(timespec="milliseconds"),
            "frame_timestamp_sec": float(frame.timestamp_sec),
            "depth_scale_m": float(frame.depth_scale_m),
            "files": files,
            "error": error,
        }
        if detection is not None:
            data["detection"] = {
                "class_name": str(detection.class_name),
                "confidence": float(detection.confidence),
                "bbox_xyxy": [
                    int(detection.x_min),
                    int(detection.y_min),
                    int(detection.x_max),
                    int(detection.y_max),
                ],
            }
        if point_cloud is not None:
            data["yolo_roi_cloud"] = {
                "point_count": int(point_cloud.points_camera_m.shape[0]),
                "roi_xyxy": [int(value) for value in point_cloud.roi_xyxy],
            }
        if processing is not None:
            support = processing.support_plane
            data["processing"] = {
                "filtered_point_count": int(
                    processing.filtered_cloud.points_camera_m.shape[0]
                ),
                "candidate_point_count": int(
                    processing.candidate_cloud.points_camera_m.shape[0]
                ),
                "final_point_count": int(
                    processing.selected_cloud.points_camera_m.shape[0]
                ),
                "sor_removed_count": int(processing.sor_removed_count),
                "cluster_count": len(processing.clusters),
                "support_plane": None
                if support is None
                else {
                    "valid": True,
                    "inlier_count": int(support.inlier_count),
                    "inlier_ratio": float(support.inlier_ratio),
                    "normal_camera": [float(value) for value in support.normal_camera],
                    "offset": float(support.offset),
                    "normal_angle_deg": float(support.normal_angle_deg),
                },
            }
        if estimate is not None:
            data["estimate"] = {
                "center_camera_m": [float(value) for value in estimate.center_camera_m],
                "center_base_m": [float(value) for value in estimate.center_base_m],
                "confidence": float(estimate.confidence),
                "point_count": int(estimate.point_count),
                "support_plane_valid": bool(estimate.support_plane_valid),
                "surface_rmse_m": float(estimate.surface_rmse_m),
                "surface_inlier_ratio": float(estimate.surface_inlier_ratio),
            }
        return data

    def _remove_old_snapshots(self) -> None:
        directories = [path for path in self._root_dir.iterdir() if path.is_dir()]
        directories.sort(key=lambda path: path.stat().st_mtime_ns, reverse=True)
        for path in directories[self._max_snapshots :]:
            shutil.rmtree(path)
