"""Direct RealSense SDK RGB-D acquisition."""

from __future__ import annotations

from dataclasses import dataclass
import threading
import time
from typing import Any

import numpy as np


class RealSenseCameraError(RuntimeError):
    """Raised when the camera cannot start or return a valid aligned frame."""


@dataclass(frozen=True)
class AlignedRgbdFrame:
    """One color frame and one depth frame aligned to the color image."""

    color_bgr: Any
    depth_u16: Any
    color_intrinsics: Any
    depth_scale_m: float
    timestamp_sec: float


class RealSenseCamera:
    """Owns one SDK pipeline and aligns depth pixels to the color image."""

    def __init__(self, parameters: dict[str, Any]) -> None:
        self._serial_no = str(parameters["camera_serial_no"])
        self._width = int(parameters["camera_width"])
        self._height = int(parameters["camera_height"])
        self._fps = int(parameters["camera_fps"])
        self._warmup_frames = int(parameters["camera_warmup_frames"])
        self._request_discard_frames = int(
            parameters.get("camera_request_discard_frames", 0)
        )
        self._startup_timeout_ms = int(parameters["camera_startup_timeout_ms"])
        self._frame_timeout_ms = int(parameters["camera_frame_timeout_ms"])
        self._keep_running = bool(parameters["camera_keep_running"])
        self._reset_on_start_failure = bool(
            parameters["camera_reset_on_start_failure"]
        )
        self._reset_reconnect_timeout_ms = int(
            parameters["camera_reset_reconnect_timeout_ms"]
        )
        self._reset_settle_ms = int(parameters["camera_reset_settle_ms"])
        self._validate_parameters()

        self._lock = threading.Lock()
        self._rs: Any | None = None
        self._pipeline: Any | None = None
        self._alignment: Any | None = None
        self._depth_scale_m = 0.0
        self._started = False

    @property
    def started(self) -> bool:
        return self._started

    def start(self) -> None:
        """Start both streams and wait for auto-exposure/depth to settle."""
        with self._lock:
            self._start_locked()

    def capture_aligned(self) -> AlignedRgbdFrame:
        """Capture one depth frame aligned to the corresponding color frame."""
        with self._lock:
            self._start_locked()
            try:
                # Each service request gets a fresh settled frame instead of
                # reusing the first frame after a request or stream start.
                for _ in range(self._request_discard_frames):
                    self._wait_for_aligned_frames_locked()
                frames = self._wait_for_aligned_frames_locked()
                color_frame = frames.get_color_frame()
                depth_frame = frames.get_depth_frame()
                if not color_frame or not depth_frame:
                    raise RealSenseCameraError(
                        "RealSense returned an incomplete aligned RGB-D frameset"
                    )

                color_bgr = np.asanyarray(color_frame.get_data()).copy()
                depth_u16 = np.asanyarray(depth_frame.get_data()).copy()
                if color_bgr.ndim != 3 or color_bgr.shape[2] != 3:
                    raise RealSenseCameraError(
                        f"Unexpected color image shape: {color_bgr.shape}"
                    )
                if depth_u16.ndim != 2 or depth_u16.dtype != np.uint16:
                    raise RealSenseCameraError(
                        f"Unexpected depth image shape/type: {depth_u16.shape}/{depth_u16.dtype}"
                    )
                if color_bgr.shape[:2] != depth_u16.shape:
                    raise RealSenseCameraError(
                        "Aligned color and depth image dimensions do not match"
                    )

                intrinsics = (
                    depth_frame.profile.as_video_stream_profile().get_intrinsics()
                )
                return AlignedRgbdFrame(
                    color_bgr=color_bgr,
                    depth_u16=depth_u16,
                    color_intrinsics=intrinsics,
                    depth_scale_m=self._depth_scale_m,
                    timestamp_sec=float(depth_frame.get_timestamp()) * 0.001,
                )
            except RealSenseCameraError:
                raise
            except Exception as error:
                raise RealSenseCameraError(
                    f"Failed to capture aligned RealSense RGB-D frame: {error}"
                ) from error
            finally:
                if not self._keep_running:
                    self._stop_locked()

    def close(self) -> None:
        """Release the SDK pipeline and the D435 device."""
        with self._lock:
            self._stop_locked()

    def _start_locked(self) -> None:
        if self._started:
            return
        try:
            self._start_once_locked()
        except RealSenseCameraError as first_error:
            if not self._should_reset_after_start_failure(first_error):
                raise
            try:
                self._hardware_reset_and_wait_locked()
                self._start_once_locked()
            except Exception as retry_error:
                raise RealSenseCameraError(
                    f"{first_error}; automatic hardware-reset retry failed: {retry_error}"
                ) from retry_error

    def _start_once_locked(self) -> None:
        pipeline: Any | None = None
        try:
            import pyrealsense2 as rs

            pipeline = rs.pipeline()
            stream_config = rs.config()
            if self._serial_no:
                stream_config.enable_device(self._serial_no)
            stream_config.enable_stream(
                rs.stream.color,
                self._width,
                self._height,
                rs.format.bgr8,
                self._fps,
            )
            stream_config.enable_stream(
                rs.stream.depth,
                self._width,
                self._height,
                rs.format.z16,
                self._fps,
            )
            profile = pipeline.start(stream_config)
            depth_sensor = profile.get_device().first_depth_sensor()
            depth_scale_m = float(depth_sensor.get_depth_scale())
            if not np.isfinite(depth_scale_m) or depth_scale_m <= 0.0:
                pipeline.stop()
                raise RealSenseCameraError(
                    f"Invalid RealSense depth scale: {depth_scale_m}"
                )

            self._rs = rs
            self._pipeline = pipeline
            self._alignment = rs.align(rs.stream.color)
            self._depth_scale_m = depth_scale_m
            self._started = True

            for frame_index in range(self._warmup_frames):
                timeout_ms = (
                    self._startup_timeout_ms
                    if frame_index == 0
                    else self._frame_timeout_ms
                )
                self._wait_for_aligned_frames_locked(timeout_ms)
        except RealSenseCameraError:
            self._cleanup_failed_start_locked(pipeline)
            raise
        except Exception as error:
            self._cleanup_failed_start_locked(pipeline)
            serial_text = self._serial_no or "first available device"
            raise RealSenseCameraError(
                f"Failed to start RealSense ({serial_text}): {error}"
            ) from error

    def _wait_for_aligned_frames_locked(self, timeout_ms: int | None = None) -> Any:
        if self._pipeline is None or self._alignment is None:
            raise RealSenseCameraError("RealSense pipeline is not started")
        frames = self._pipeline.wait_for_frames(timeout_ms or self._frame_timeout_ms)
        aligned_frames = self._alignment.process(frames)
        if not aligned_frames:
            raise RealSenseCameraError("RealSense depth-to-color alignment failed")
        return aligned_frames

    def _stop_locked(self) -> None:
        pipeline = self._pipeline
        self._reset_state_locked()
        if pipeline is None:
            return
        try:
            pipeline.stop()
        except Exception as error:
            raise RealSenseCameraError(f"Failed to stop RealSense pipeline: {error}") from error

    def _reset_state_locked(self) -> None:
        self._pipeline = None
        self._alignment = None
        self._rs = None
        self._depth_scale_m = 0.0
        self._started = False

    def _cleanup_failed_start_locked(self, pipeline: Any | None) -> None:
        self._reset_state_locked()
        if pipeline is None:
            return
        try:
            pipeline.stop()
        except Exception:
            pass

    def _should_reset_after_start_failure(self, error: Exception) -> bool:
        return (
            self._reset_on_start_failure
            and "Frame didn't arrive" in str(error)
        )

    def _hardware_reset_and_wait_locked(self) -> None:
        import pyrealsense2 as rs

        devices = rs.context().query_devices()
        if len(devices) == 0:
            raise RealSenseCameraError("No RealSense device is available for hardware reset")

        selected_device = None
        selected_serial = ""
        for device in devices:
            serial = device.get_info(rs.camera_info.serial_number)
            if not self._serial_no or serial == self._serial_no:
                selected_device = device
                selected_serial = serial
                break
        if selected_device is None:
            raise RealSenseCameraError(
                f"Configured RealSense serial was not found: {self._serial_no}"
            )

        selected_device.hardware_reset()
        del selected_device, device, devices

        deadline = time.monotonic() + self._reset_reconnect_timeout_ms * 0.001
        while time.monotonic() < deadline:
            time.sleep(0.25)
            try:
                reconnected_devices = rs.context().query_devices()
                for device in reconnected_devices:
                    serial = device.get_info(rs.camera_info.serial_number)
                    if serial == selected_serial:
                        time.sleep(self._reset_settle_ms * 0.001)
                        return
            except Exception:
                continue
        raise RealSenseCameraError(
            f"RealSense {selected_serial} did not reconnect within "
            f"{self._reset_reconnect_timeout_ms} ms"
        )

    def _validate_parameters(self) -> None:
        if self._width <= 0 or self._height <= 0 or self._fps <= 0:
            raise ValueError("camera_width, camera_height and camera_fps must be positive")
        if self._warmup_frames < 0:
            raise ValueError("camera_warmup_frames cannot be negative")
        if self._request_discard_frames < 0:
            raise ValueError("camera_request_discard_frames cannot be negative")
        if self._startup_timeout_ms <= 0:
            raise ValueError("camera_startup_timeout_ms must be positive")
        if self._frame_timeout_ms <= 0:
            raise ValueError("camera_frame_timeout_ms must be positive")
        if self._reset_reconnect_timeout_ms <= 0:
            raise ValueError("camera_reset_reconnect_timeout_ms must be positive")
        if self._reset_settle_ms < 0:
            raise ValueError("camera_reset_settle_ms cannot be negative")
