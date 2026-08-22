"""YOLOv8-World single-crate detection."""

from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path
import threading
from typing import Any, Sequence

import numpy as np


class YoloWorldDetectorError(RuntimeError):
    """Raised when the YOLO model cannot load or complete inference."""


@dataclass(frozen=True)
class BoxDetection:
    """Selected YOLO detection in aligned color-image coordinates."""

    x_min: int
    y_min: int
    x_max: int
    y_max: int
    confidence: float
    class_name: str


class YoloWorldDetector:
    """Lazily loads YOLO-World and selects one physical crate detection."""

    def __init__(
        self,
        model_path: str,
        class_prompts: Sequence[str],
        parameters: dict[str, Any],
    ) -> None:
        self._model_path = Path(model_path).expanduser()
        self._class_prompts = self._normalize_prompts(class_prompts)
        self._device = self._resolve_device(str(parameters["yolo_device"]))
        self._image_size = int(parameters["yolo_image_size"])
        self._confidence_threshold = float(
            parameters["yolo_confidence_threshold"]
        )
        self._iou_threshold = float(parameters["yolo_iou_threshold"])
        self._agnostic_nms = bool(parameters["yolo_agnostic_nms"])
        self._max_detections = int(parameters["yolo_max_detections"])
        self._min_bbox_width_px = int(parameters["yolo_min_bbox_width_px"])
        self._min_bbox_height_px = int(parameters["yolo_min_bbox_height_px"])
        self._validate_parameters()

        self._model: Any | None = None
        self._load_lock = threading.Lock()

    @property
    def loaded(self) -> bool:
        return self._model is not None

    @property
    def model_path(self) -> Path:
        return self._model_path

    @property
    def class_prompts(self) -> tuple[str, ...]:
        return self._class_prompts

    def detect_one(self, color_bgr: Any) -> BoxDetection | None:
        image = self._validate_image(color_bgr)
        model = self._load_model()
        try:
            results = model.predict(
                source=image,
                conf=self._confidence_threshold,
                iou=self._iou_threshold,
                imgsz=self._image_size,
                device=self._device,
                max_det=self._max_detections,
                agnostic_nms=self._agnostic_nms,
                verbose=False,
            )
        except Exception as error:
            raise YoloWorldDetectorError(
                f"YOLOv8-World inference failed: {error}"
            ) from error

        if not results or results[0].boxes is None or len(results[0].boxes) == 0:
            return None

        boxes = results[0].boxes
        xyxy = boxes.xyxy.detach().cpu().numpy()
        confidences = boxes.conf.detach().cpu().numpy()
        class_indices = boxes.cls.detach().cpu().numpy().astype(np.int64)
        order = np.argsort(-confidences)
        image_height, image_width = image.shape[:2]

        for index in order:
            detection = self._make_detection(
                xyxy[index],
                float(confidences[index]),
                int(class_indices[index]),
                image_width,
                image_height,
            )
            if detection is not None:
                return detection
        return None

    def _load_model(self) -> Any:
        if self._model is not None:
            return self._model
        with self._load_lock:
            if self._model is not None:
                return self._model
            if not self._model_path.is_file():
                raise YoloWorldDetectorError(
                    f"YOLOv8-World weights do not exist: {self._model_path}"
                )
            try:
                from ultralytics import YOLOWorld

                model = YOLOWorld(str(self._model_path))
                model.set_classes(list(self._class_prompts))
            except Exception as error:
                raise YoloWorldDetectorError(
                    f"Failed to load YOLOv8-World model {self._model_path}: {error}"
                ) from error
            self._model = model
            return model

    def _make_detection(
        self,
        xyxy: np.ndarray,
        confidence: float,
        class_index: int,
        image_width: int,
        image_height: int,
    ) -> BoxDetection | None:
        if xyxy.shape != (4,) or not np.all(np.isfinite(xyxy)):
            return None
        if not math.isfinite(confidence):
            return None

        x_min = max(0, min(image_width, int(math.floor(float(xyxy[0])))))
        y_min = max(0, min(image_height, int(math.floor(float(xyxy[1])))))
        x_max = max(0, min(image_width, int(math.ceil(float(xyxy[2])))))
        y_max = max(0, min(image_height, int(math.ceil(float(xyxy[3])))))
        if (
            x_max - x_min < self._min_bbox_width_px
            or y_max - y_min < self._min_bbox_height_px
        ):
            return None
        if class_index < 0 or class_index >= len(self._class_prompts):
            return None
        return BoxDetection(
            x_min=x_min,
            y_min=y_min,
            x_max=x_max,
            y_max=y_max,
            confidence=confidence,
            class_name=self._class_prompts[class_index],
        )

    @staticmethod
    def _validate_image(color_bgr: Any) -> np.ndarray:
        if not isinstance(color_bgr, np.ndarray):
            raise YoloWorldDetectorError("YOLO input must be a NumPy image")
        if color_bgr.ndim != 3 or color_bgr.shape[2] != 3:
            raise YoloWorldDetectorError(
                f"YOLO input must have HxWx3 shape, got {color_bgr.shape}"
            )
        if color_bgr.dtype != np.uint8:
            raise YoloWorldDetectorError(
                f"YOLO input must use uint8 BGR pixels, got {color_bgr.dtype}"
            )
        if color_bgr.shape[0] <= 0 or color_bgr.shape[1] <= 0:
            raise YoloWorldDetectorError("YOLO input image is empty")
        return color_bgr

    @staticmethod
    def _normalize_prompts(class_prompts: Sequence[str]) -> tuple[str, ...]:
        prompts: list[str] = []
        for value in class_prompts:
            prompt = str(value).strip()
            if prompt and prompt not in prompts:
                prompts.append(prompt)
        if not prompts:
            raise ValueError("yolo_class_prompts must contain at least one non-empty prompt")
        return tuple(prompts)

    @staticmethod
    def _resolve_device(configured_device: str) -> str:
        device = configured_device.strip().lower()
        if device != "auto":
            return configured_device.strip()
        try:
            import torch

            return "0" if torch.cuda.is_available() else "cpu"
        except Exception:
            return "cpu"

    def _validate_parameters(self) -> None:
        if not self._device:
            raise ValueError("yolo_device cannot be empty")
        if self._image_size <= 0:
            raise ValueError("yolo_image_size must be positive")
        if not 0.0 <= self._confidence_threshold <= 1.0:
            raise ValueError("yolo_confidence_threshold must be in [0, 1]")
        if not 0.0 <= self._iou_threshold <= 1.0:
            raise ValueError("yolo_iou_threshold must be in [0, 1]")
        if self._max_detections <= 0:
            raise ValueError("yolo_max_detections must be positive")
        if self._min_bbox_width_px <= 0 or self._min_bbox_height_px <= 0:
            raise ValueError("YOLO minimum bbox dimensions must be positive")
