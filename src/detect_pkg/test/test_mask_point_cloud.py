from types import SimpleNamespace

import numpy as np
import pytest

from detect_box_pipeline.box_position_estimator import (
    BoxPositionEstimator,
    PointCloudExtractionError,
)
from detect_box_pipeline.debug_snapshot import DebugSnapshotWriter
from detect_box_pipeline.yoloe_segmenter import (
    YoloeSegmenter,
    YoloeSegmenterError,
)


class _FakeTensor:
    def __init__(self, array: np.ndarray) -> None:
        self._array = np.asarray(array)
        self.device = "cuda:0"

    def __len__(self) -> int:
        return len(self._array)

    def detach(self) -> "_FakeTensor":
        return self

    def cpu(self) -> "_FakeTensor":
        return self

    def numpy(self) -> np.ndarray:
        return self._array


class _FakeModel:
    def __init__(self, result: SimpleNamespace) -> None:
        self._result = result

    def predict(self, **_: object) -> list[SimpleNamespace]:
        return [self._result]


def _segmenter() -> YoloeSegmenter:
    return YoloeSegmenter(
        "/tmp/not-loaded-yoloe.pt",
        ["green plastic crate", "plastic crate", "plastic crate"],
        {
            "yolo_device": "cpu",
            "yolo_image_size": 640,
            "yolo_confidence_threshold": 0.1,
            "yolo_iou_threshold": 0.45,
            "yolo_agnostic_nms": True,
            "yolo_max_detections": 10,
            "yolo_min_bbox_width_px": 2,
            "yolo_min_bbox_height_px": 2,
            "yolo_mask_threshold": 0.5,
            "yolo_min_mask_area_px": 2,
            "yolo_half_precision": True,
        },
    )


def _estimator(mask_erosion_px: int = 0) -> BoxPositionEstimator:
    estimator = BoxPositionEstimator.__new__(BoxPositionEstimator)
    estimator._depth_min_m = 0.2
    estimator._depth_max_m = 3.0
    estimator._bbox_margin_px = 0
    estimator._mask_erosion_px = mask_erosion_px
    estimator._pixel_stride = 1
    estimator._min_points = 1
    return estimator


def _frame(size: int) -> SimpleNamespace:
    return SimpleNamespace(
        color_bgr=np.zeros((size, size, 3), dtype=np.uint8),
        depth_u16=np.full((size, size), 1000, dtype=np.uint16),
        depth_scale_m=0.001,
        color_intrinsics=SimpleNamespace(fx=1.0, fy=1.0, ppx=0.0, ppy=0.0),
    )


def _detection(mask: np.ndarray) -> SimpleNamespace:
    height, width = mask.shape
    return SimpleNamespace(
        x_min=0,
        y_min=0,
        x_max=width,
        y_max=height,
        mask=mask,
    )


def test_segmenter_builds_full_resolution_boolean_mask() -> None:
    segmenter = _segmenter()
    mask = np.array(
        [
            [0.1, 0.8, 0.2, 0.1],
            [0.1, 0.9, 0.7, 0.1],
            [0.1, 0.1, 0.1, 0.1],
        ],
        dtype=np.float32,
    )

    result = segmenter._make_segmentation(
        np.array([0.0, 0.0, 4.0, 3.0]),
        0.8,
        0,
        mask,
        image_width=4,
        image_height=3,
    )

    assert result is not None
    assert result.mask.dtype == np.bool_
    assert result.mask.shape == (3, 4)
    assert int(np.count_nonzero(result.mask)) == 3
    assert segmenter.class_prompts == ("green plastic crate", "plastic crate")


def test_segmenter_rejects_non_source_sized_mask() -> None:
    with pytest.raises(YoloeSegmenterError, match="does not match"):
        _segmenter()._make_segmentation(
            np.array([0.0, 0.0, 4.0, 3.0]),
            0.8,
            0,
            np.ones((2, 2), dtype=np.float32),
            image_width=4,
            image_height=3,
        )


def test_segmenter_keeps_box_and_mask_indices_aligned() -> None:
    low_mask = np.zeros((4, 4), dtype=np.float32)
    low_mask[0, 0:2] = 1.0
    high_mask = np.zeros((4, 4), dtype=np.float32)
    high_mask[2:4, 2:4] = 1.0
    result = SimpleNamespace(
        boxes=SimpleNamespace(
            xyxy=_FakeTensor(
                np.array([[0.0, 0.0, 2.0, 2.0], [1.0, 1.0, 4.0, 4.0]])
            ),
            conf=_FakeTensor(np.array([0.3, 0.9])),
            cls=_FakeTensor(np.array([0.0, 1.0])),
            __len__=lambda: 2,
        ),
        masks=SimpleNamespace(data=_FakeTensor(np.stack((low_mask, high_mask)))),
    )
    # Special methods are looked up on the type rather than the instance.
    result.boxes = type(
        "FakeBoxes",
        (),
        {
            "__len__": lambda self: 2,
            "xyxy": result.boxes.xyxy,
            "conf": result.boxes.conf,
            "cls": result.boxes.cls,
        },
    )()
    segmenter = _segmenter()
    segmenter._model = _FakeModel(result)

    selected = segmenter.segment_one(np.zeros((4, 4, 3), dtype=np.uint8))

    assert selected is not None
    assert selected.class_name == "plastic crate"
    assert selected.confidence == pytest.approx(0.9)
    np.testing.assert_array_equal(selected.mask, high_mask.astype(bool))


def test_point_cloud_contains_only_masked_depth_pixels() -> None:
    mask = np.zeros((4, 4), dtype=bool)
    mask[1, 1] = True
    mask[2, 2] = True

    cloud = _estimator().extract_point_cloud(_frame(4), _detection(mask))

    np.testing.assert_array_equal(
        cloud.pixels_uv,
        np.array([[1.0, 1.0], [2.0, 2.0]], dtype=np.float32),
    )
    np.testing.assert_allclose(
        cloud.points_camera_m,
        np.array([[1.0, 1.0, 1.0], [2.0, 2.0, 1.0]], dtype=np.float32),
    )


def test_point_cloud_erodes_only_the_sampling_mask() -> None:
    mask = np.zeros((5, 5), dtype=bool)
    mask[1:4, 1:4] = True

    cloud = _estimator(mask_erosion_px=1).extract_point_cloud(
        _frame(5),
        _detection(mask),
    )

    np.testing.assert_array_equal(
        cloud.pixels_uv,
        np.array([[2.0, 2.0]], dtype=np.float32),
    )


def test_point_cloud_rejects_mismatched_mask_shape() -> None:
    with pytest.raises(PointCloudExtractionError, match="mask shape"):
        _estimator().extract_point_cloud(
            _frame(4),
            _detection(np.ones((3, 3), dtype=bool)),
        )


def test_debug_snapshot_writes_only_requested_artifacts(tmp_path) -> None:
    writer = DebugSnapshotWriter(
        {
            "debug_enabled": True,
            "debug_output_dir": str(tmp_path),
            "debug_max_snapshots": 2,
        }
    )
    mask = np.ones((4, 4), dtype=bool)
    detection = SimpleNamespace(
        x_min=0,
        y_min=0,
        x_max=4,
        y_max=4,
        confidence=0.9,
        class_name="plastic crate",
        mask=mask,
    )
    point_cloud = SimpleNamespace(
        points_camera_m=np.array([[0.0, 0.0, 1.0], [0.1, 0.1, 1.0]]),
        pixels_uv=np.array([[0.0, 0.0], [1.0, 1.0]]),
    )

    snapshot = writer.save(
        _frame(4),
        detection=detection,
        point_cloud=point_cloud,
    )

    assert snapshot is not None
    assert sorted(path.name for path in snapshot.iterdir()) == [
        "color_with_yoloe_mask.png",
        "mask_roi_cloud.ply",
    ]
