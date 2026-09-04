import threading
import time

from cv_bridge import CvBridge
import numpy as np
import pytest
from sensor_msgs.msg import CameraInfo

from detect_box_sim_pipeline.topic_rgbd_camera import (
    AlignedRgbdFrame,
    CameraIntrinsics,
    TopicRgbdCamera,
    TopicRgbdCameraError,
)


def _camera_without_subscriptions() -> TopicRgbdCamera:
    camera = TopicRgbdCamera.__new__(TopicRgbdCamera)
    camera._bridge = CvBridge()
    return camera


def _messages():
    bridge = CvBridge()
    rgb = np.array([[[255, 0, 0], [0, 255, 0]]], dtype=np.uint8)
    depth = np.array([[0.75, 1.25]], dtype=np.float32)
    color_msg = bridge.cv2_to_imgmsg(rgb, encoding="rgb8")
    depth_msg = bridge.cv2_to_imgmsg(depth, encoding="32FC1")
    color_msg.header.stamp.sec = 12
    color_msg.header.stamp.nanosec = 300_000_000
    depth_msg.header = color_msg.header

    camera_info = CameraInfo()
    camera_info.header = color_msg.header
    camera_info.width = 2
    camera_info.height = 1
    camera_info.k = [500.0, 0.0, 1.0, 0.0, 501.0, 0.5, 0.0, 0.0, 1.0]
    return color_msg, depth_msg, camera_info


def test_rgb8_and_metric_depth_conversion() -> None:
    camera = _camera_without_subscriptions()
    frame = camera._convert_frame(*_messages())

    np.testing.assert_array_equal(
        frame.color_bgr,
        np.array([[[0, 0, 255], [0, 255, 0]]], dtype=np.uint8),
    )
    np.testing.assert_allclose(
        frame.depth_image,
        np.array([[0.75, 1.25]], dtype=np.float32),
    )
    assert frame.depth_scale_m == 1.0
    assert frame.timestamp_sec == pytest.approx(12.3)
    assert frame.color_intrinsics == CameraIntrinsics(500.0, 501.0, 1.0, 0.5)


def test_wrong_depth_encoding_is_rejected() -> None:
    camera = _camera_without_subscriptions()
    color_msg, depth_msg, camera_info = _messages()
    depth_msg.encoding = "16UC1"

    with pytest.raises(TopicRgbdCameraError, match="32FC1"):
        camera._convert_frame(color_msg, depth_msg, camera_info)


def test_capture_waits_for_a_frame_newer_than_the_request() -> None:
    camera = TopicRgbdCamera.__new__(TopicRgbdCamera)
    camera._condition = threading.Condition()
    camera._latest_sequence = 1
    camera._wait_timeout_sec = 0.5
    camera._last_error = ""
    camera._color_topic = "/color"
    camera._aligned_depth_topic = "/depth"
    camera._camera_info_topic = "/camera_info"
    old_frame = AlignedRgbdFrame(
        color_bgr=np.zeros((1, 1, 3), dtype=np.uint8),
        depth_image=np.ones((1, 1), dtype=np.float32),
        color_intrinsics=CameraIntrinsics(1.0, 1.0, 0.0, 0.0),
        depth_scale_m=1.0,
        timestamp_sec=1.0,
    )
    new_frame = AlignedRgbdFrame(
        color_bgr=np.ones((1, 1, 3), dtype=np.uint8),
        depth_image=np.full((1, 1), 2.0, dtype=np.float32),
        color_intrinsics=CameraIntrinsics(1.0, 1.0, 0.0, 0.0),
        depth_scale_m=1.0,
        timestamp_sec=2.0,
    )
    camera._latest_frame = old_frame

    def publish_new_frame() -> None:
        time.sleep(0.05)
        with camera._condition:
            camera._latest_frame = new_frame
            camera._latest_sequence += 1
            camera._condition.notify_all()

    publisher = threading.Thread(target=publish_new_frame)
    publisher.start()
    captured = camera.capture_aligned()
    publisher.join()

    assert captured is new_frame
