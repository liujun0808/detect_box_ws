"""Per-request RGB-D, bounding-box and point-cloud debug snapshots."""

from __future__ import annotations

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
    ) -> Path | None:
        """Save available data for one request and return its directory."""
        if not self._enabled or self._root_dir is None:
            return None

        self._root_dir.mkdir(parents=True, exist_ok=True)
        snapshot_dir = self._new_snapshot_dir()
        snapshot_dir.mkdir()

        color = np.asarray(frame.color_bgr)
        if detection is not None:
            annotated = color.copy()
            self._draw_detection(annotated, detection)
            self._write_image(snapshot_dir / "color_with_yolo_bbox.png", annotated)

        depth_u16 = np.asarray(frame.depth_u16)
        self._write_image(snapshot_dir / "depth_u16.png", depth_u16)

        if point_cloud is not None:
            self._write_ply(
                snapshot_dir / "yolo_roi_cloud.ply",
                point_cloud.points_camera_m,
                self._cloud_colors_bgr(frame, point_cloud),
            )
        if processing is not None:
            self._write_ply(
                snapshot_dir / "final_candidate_cloud.ply",
                processing.selected_cloud.points_camera_m,
                self._cloud_colors_bgr(frame, processing.selected_cloud),
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
        color = (0, 255, 0)
        cv2.rectangle(image, (x_min, y_min), (x_max, y_max), color, 3)
        label = f"{detection.class_name}  conf={float(detection.confidence):.3f}"
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.65
        thickness = 2
        (text_width, text_height), baseline = cv2.getTextSize(
            label,
            font,
            font_scale,
            thickness,
        )
        label_top = y_min - text_height - baseline - 8
        if label_top < 0:
            label_top = min(height - text_height - baseline - 8, y_min + 4)
        label_top = max(0, label_top)
        label_bottom = min(height - 1, label_top + text_height + baseline + 8)
        cv2.rectangle(
            image,
            (x_min, label_top),
            (min(width - 1, x_min + text_width + 8), label_bottom),
            color,
            cv2.FILLED,
        )
        cv2.putText(
            image,
            label,
            (x_min + 4, label_bottom - baseline - 4),
            font,
            font_scale,
            (0, 0, 0),
            thickness,
            cv2.LINE_AA,
        )

    @staticmethod
    def _cloud_colors_bgr(frame: Any, point_cloud: Any) -> np.ndarray:
        color = np.asarray(frame.color_bgr)
        pixels = np.rint(np.asarray(point_cloud.pixels_uv)).astype(np.int64)
        colors = np.zeros((pixels.shape[0], 3), dtype=np.uint8)
        height, width = color.shape[:2]
        valid = (
            (pixels[:, 0] >= 0)
            & (pixels[:, 0] < width)
            & (pixels[:, 1] >= 0)
            & (pixels[:, 1] < height)
        )
        colors[valid] = color[pixels[valid, 1], pixels[valid, 0]]
        return colors

    @staticmethod
    def _write_ply(
        path: Path,
        points: Any,
        colors_bgr: Any | None = None,
    ) -> None:
        array = np.asarray(points, dtype=np.float64)
        if array.size == 0:
            array = np.empty((0, 3), dtype=np.float64)
        if array.ndim != 2 or array.shape[1] != 3:
            raise ValueError(f"Point cloud must have shape (N, 3), got {array.shape}")
        finite = np.all(np.isfinite(array), axis=1)
        array = array[finite]
        colors = None
        if colors_bgr is not None:
            colors = np.asarray(colors_bgr, dtype=np.uint8)
            if colors.shape != (finite.shape[0], 3):
                raise ValueError(
                    "Point-cloud colors must have shape (N, 3), "
                    f"got {colors.shape}"
                )
            colors = colors[finite]
        with path.open("w", encoding="ascii") as stream:
            stream.write("ply\nformat ascii 1.0\n")
            stream.write(f"element vertex {array.shape[0]}\n")
            stream.write("property float x\nproperty float y\nproperty float z\n")
            if colors is not None:
                stream.write(
                    "property uchar red\n"
                    "property uchar green\n"
                    "property uchar blue\n"
                )
            stream.write("end_header\n")
            if colors is None:
                for x, y, z in array:
                    stream.write(f"{x:.7f} {y:.7f} {z:.7f}\n")
            else:
                for (x, y, z), (blue, green, red) in zip(array, colors):
                    stream.write(
                        f"{x:.7f} {y:.7f} {z:.7f} "
                        f"{int(red)} {int(green)} {int(blue)}\n"
                    )

    def _remove_old_snapshots(self) -> None:
        directories = [path for path in self._root_dir.iterdir() if path.is_dir()]
        directories.sort(key=lambda path: path.stat().st_mtime_ns, reverse=True)
        for path in directories[self._max_snapshots :]:
            shutil.rmtree(path)
