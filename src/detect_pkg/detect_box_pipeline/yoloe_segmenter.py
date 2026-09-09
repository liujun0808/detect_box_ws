"""YOLOE-26 promptable instance segmentation for one physical crate."""

from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path
import threading
from typing import Any, Sequence

import numpy as np


class YoloeSegmenterError(RuntimeError):
    """Raised when the YOLOE model cannot load or complete inference."""


@dataclass(frozen=True)
class BoxSegmentation:
    """One selected crate instance in aligned color-image coordinates."""

    x_min: int
    y_min: int
    x_max: int
    y_max: int
    confidence: float
    class_name: str
    mask: np.ndarray


class YoloeSegmenter:
    """Load YOLOE-26, apply text prompts and select one crate instance."""

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
        self._mask_threshold = float(parameters["yolo_mask_threshold"])
        self._min_mask_area_px = int(parameters["yolo_min_mask_area_px"])
        self._half_precision = bool(parameters["yolo_half_precision"])
        self._validate_parameters()

        self._model: Any | None = None
        self._load_lock = threading.Lock()
        self._last_inference_device: str | None = None

    @property
    def loaded(self) -> bool:
        return self._model is not None

    @property
    def model_path(self) -> Path:
        return self._model_path

    @property
    def class_prompts(self) -> tuple[str, ...]:
        return self._class_prompts

    @property
    def resolved_device(self) -> str:
        return self._device

    @property
    def resolved_device_label(self) -> str:
        device = self._device.strip().lower()
        if device.isdigit():
            return f"cuda:{device}"
        if device == "cuda":
            return "cuda:0"
        return self._device

    @property
    def last_inference_device(self) -> str | None:
        return self._last_inference_device

    def initialize(self, warmup: bool = True) -> None:
        """Load YOLOE/text embeddings and optionally run one warmup frame."""
        model = self._load_model()
        self._configure_cuda_backend()
        if not warmup:
            return

        warmup_image = np.zeros(
            (self._image_size, self._image_size, 3),
            dtype=np.uint8,
        )
        try:
            model.predict(
                source=warmup_image,
                imgsz=self._image_size,
                device=self._device,
                half=self._use_half_precision(),
                max_det=self._max_detections,
                agnostic_nms=self._agnostic_nms,
                retina_masks=True,
                verbose=False,
            )
        except Exception as error:
            raise YoloeSegmenterError(
                f"YOLOE-26 startup warmup failed: {error}"
            ) from error

    def segment_one(self, color_bgr: Any) -> BoxSegmentation | None:
        """Return the highest-confidence valid prompted crate instance."""
        image = self._validate_image(color_bgr)
        model = self._load_model()
        try:
            results = model.predict(
                source=image,
                conf=self._confidence_threshold,
                iou=self._iou_threshold,
                imgsz=self._image_size,
                device=self._device,
                half=self._use_half_precision(),
                max_det=self._max_detections,
                agnostic_nms=self._agnostic_nms,
                retina_masks=True,
                verbose=False,
            )
        except Exception as error:
            raise YoloeSegmenterError(
                f"YOLOE-26 segmentation failed: {error}"
            ) from error

        if not results:
            return None
        result = results[0]
        if result.boxes is None or len(result.boxes) == 0:
            return None
        if result.masks is None or result.masks.data is None:
            raise YoloeSegmenterError(
                "YOLOE-26 returned boxes without instance masks"
            )

        boxes = result.boxes
        masks = result.masks.data
        if len(masks) != len(boxes):
            raise YoloeSegmenterError(
                "YOLOE-26 box/mask count mismatch: "
                f"boxes={len(boxes)}, masks={len(masks)}"
            )
        try:
            self._last_inference_device = str(boxes.xyxy.device)
        except Exception:
            self._last_inference_device = self.resolved_device_label

        xyxy = boxes.xyxy.detach().cpu().numpy()
        confidences = boxes.conf.detach().cpu().numpy()
        class_indices = boxes.cls.detach().cpu().numpy().astype(np.int64)
        mask_arrays = masks.detach().cpu().numpy()
        order = np.argsort(-confidences)
        image_height, image_width = image.shape[:2]

        for index in order:
            segmentation = self._make_segmentation(
                xyxy[index],
                float(confidences[index]),
                int(class_indices[index]),
                mask_arrays[index],
                image_width,
                image_height,
            )
            if segmentation is not None:
                return segmentation
        return None

    def _load_model(self) -> Any:
        if self._model is not None:
            return self._model
        with self._load_lock:
            if self._model is not None:
                return self._model
            if not self._model_path.is_file():
                raise YoloeSegmenterError(
                    f"YOLOE-26 weights do not exist: {self._model_path}"
                )
            try:
                from ultralytics import YOLOE

                model = YOLOE(str(self._model_path))
                model.set_classes(list(self._class_prompts))
            except Exception as error:
                raise YoloeSegmenterError(
                    f"Failed to load YOLOE-26 model {self._model_path}: {error}"
                ) from error
            self._model = model
            return model

    def _make_segmentation(
        self,
        xyxy: np.ndarray,
        confidence: float,
        class_index: int,
        mask: np.ndarray,
        image_width: int,
        image_height: int,
    ) -> BoxSegmentation | None:
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

        mask_array = np.asarray(mask)
        expected_shape = (image_height, image_width)
        if mask_array.shape != expected_shape:
            raise YoloeSegmenterError(
                "YOLOE-26 retina mask does not match the source image: "
                f"mask={mask_array.shape}, image={expected_shape}"
            )
        if not np.all(np.isfinite(mask_array)):
            return None
        binary_mask = np.asarray(mask_array >= self._mask_threshold, dtype=bool)
        binary_mask.setflags(write=False)
        if int(np.count_nonzero(binary_mask)) < self._min_mask_area_px:
            return None

        return BoxSegmentation(
            x_min=x_min,
            y_min=y_min,
            x_max=x_max,
            y_max=y_max,
            confidence=confidence,
            class_name=self._class_prompts[class_index],
            mask=binary_mask,
        )

    def _use_half_precision(self) -> bool:
        return self._half_precision and self.resolved_device_label != "cpu"

    def _configure_cuda_backend(self) -> None:
        """Enable stable-shape CUDA autotuning after the model is loaded."""
        if self.resolved_device_label == "cpu":
            return
        try:
            import torch

            if torch.cuda.is_available():
                # RGB-D requests always use the configured fixed input size;
                # cuDNN benchmarking avoids repeating convolution algorithm
                # selection on every request after the startup warmup.
                torch.backends.cudnn.benchmark = True
        except Exception:
            # Device selection and inference remain handled by Ultralytics;
            # this optional optimization must never prevent startup.
            return

    @staticmethod
    def _validate_image(color_bgr: Any) -> np.ndarray:
        if not isinstance(color_bgr, np.ndarray):
            raise YoloeSegmenterError("YOLOE input must be a NumPy image")
        if color_bgr.ndim != 3 or color_bgr.shape[2] != 3:
            raise YoloeSegmenterError(
                f"YOLOE input must have HxWx3 shape, got {color_bgr.shape}"
            )
        if color_bgr.dtype != np.uint8:
            raise YoloeSegmenterError(
                f"YOLOE input must use uint8 BGR pixels, got {color_bgr.dtype}"
            )
        if color_bgr.shape[0] <= 0 or color_bgr.shape[1] <= 0:
            raise YoloeSegmenterError("YOLOE input image is empty")
        return color_bgr

    @staticmethod
    def _normalize_prompts(class_prompts: Sequence[str]) -> tuple[str, ...]:
        prompts: list[str] = []
        for value in class_prompts:
            prompt = str(value).strip()
            if prompt and prompt not in prompts:
                prompts.append(prompt)
        if not prompts:
            raise ValueError(
                "yolo_class_prompts must contain at least one non-empty prompt"
            )
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
            raise ValueError("YOLOE minimum bbox dimensions must be positive")
        if not 0.0 <= self._mask_threshold <= 1.0:
            raise ValueError("yolo_mask_threshold must be in [0, 1]")
        if self._min_mask_area_px <= 0:
            raise ValueError("yolo_min_mask_area_px must be positive")
