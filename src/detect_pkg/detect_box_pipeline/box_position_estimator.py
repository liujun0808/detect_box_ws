"""Depth ROI processing and fixed-size box center estimation boundary."""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any, Sequence

import numpy as np

try:
    from scipy.optimize import least_squares
    from scipy.spatial import cKDTree
except ImportError:  # pragma: no cover - the runtime environment should provide scipy
    least_squares = None
    cKDTree = None


class PointCloudExtractionError(RuntimeError):
    """Raised when a detection does not contain enough valid depth data."""


class PointCloudProcessingError(RuntimeError):
    """Raised when filtering or clustering cannot produce a candidate cloud."""


class PositionEstimationError(RuntimeError):
    """Raised when the 3DoF position optimizer fails its quality gates."""


@dataclass(frozen=True)
class CameraPointCloud:
    """Valid depth samples from one YOLO ROI in camera coordinates."""

    points_camera_m: np.ndarray
    pixels_uv: np.ndarray
    depths_m: np.ndarray
    roi_xyxy: tuple[int, int, int, int]

    def subset(self, indices: np.ndarray) -> "CameraPointCloud":
        """Return a point-cloud view containing the selected sample indices."""
        selected = np.asarray(indices, dtype=np.int64)
        return CameraPointCloud(
            points_camera_m=self.points_camera_m[selected],
            pixels_uv=self.pixels_uv[selected],
            depths_m=self.depths_m[selected],
            roi_xyxy=self.roi_xyxy,
        )


@dataclass(frozen=True)
class PointCloudCluster:
    """Statistics for one connected 3D component after SOR filtering."""

    indices: np.ndarray
    centroid_camera_m: tuple[float, float, float]
    point_count: int
    pixel_xyxy: tuple[int, int, int, int]
    depth_min_m: float
    depth_median_m: float
    depth_max_m: float
    score: float


@dataclass(frozen=True)
class SupportPlane:
    """A validated conveyor support plane in the camera frame."""

    normal_camera: tuple[float, float, float]
    offset: float
    inlier_mask: np.ndarray
    inlier_count: int
    inlier_ratio: float
    normal_angle_deg: float


@dataclass(frozen=True)
class InnerWallPlane:
    """A validated far inner wall of the box in camera coordinates."""

    # Raw plane normal estimated from the point cloud. It is used for
    # validation only because the center optimizer keeps the nominal pose.
    normal_camera: tuple[float, float, float]
    # Fixed-pose normal derived from the configured camera/base rotation.
    nominal_normal_camera: tuple[float, float, float]
    offset: float
    inlier_mask: np.ndarray
    inlier_count: int
    inlier_ratio: float
    normal_angle_deg: float
    local_axis_index: int
    top_edge_coordinate: float | None
    top_edge_span_m: float


@dataclass(frozen=True)
class PointCloudProcessingResult:
    """Filtered cloud, cluster candidates and the selected main candidate."""

    filtered_cloud: CameraPointCloud
    candidate_cloud: CameraPointCloud
    support_plane: SupportPlane | None
    clusters: tuple[PointCloudCluster, ...]
    selected_cluster: PointCloudCluster
    selected_cloud: CameraPointCloud
    sor_removed_count: int
    inner_wall_plane: InnerWallPlane | None


@dataclass(frozen=True)
class PositionEstimate:
    """Estimated geometric center in camera and base coordinates."""

    center_camera_m: tuple[float, float, float]
    center_base_m: tuple[float, float, float]
    confidence: float
    point_count: int
    support_plane_valid: bool
    surface_rmse_m: float
    surface_inlier_ratio: float
    inner_wall_valid: bool


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
        self._sor_neighbors = int(parameters["pointcloud_statistical_neighbors"])
        self._sor_std_ratio = float(parameters["pointcloud_statistical_std_ratio"])
        self._cluster_tolerance_m = float(
            parameters["pointcloud_cluster_tolerance_m"]
        )
        self._min_cluster_points = int(parameters["pointcloud_min_cluster_points"])
        self._support_plane_threshold_m = float(
            parameters["support_plane_distance_threshold_m"]
        )
        self._support_plane_iterations = int(
            parameters["support_plane_ransac_iterations"]
        )
        self._support_plane_min_ratio = float(
            parameters["support_plane_min_inlier_ratio"]
        )
        self._support_plane_max_angle_deg = float(
            parameters["support_plane_max_normal_angle_deg"]
        )
        self._optimizer_surface_weight = float(
            parameters["optimizer_surface_weight"]
        )
        self._optimizer_containment_weight = float(
            parameters["optimizer_containment_weight"]
        )
        self._optimizer_support_weight = float(
            parameters["optimizer_support_weight"]
        )
        self._optimizer_bbox_weight = float(parameters["optimizer_bbox_weight"])
        self._optimizer_bbox_clip_transition_px = float(
            parameters.get("optimizer_bbox_clip_transition_px", 30.0)
        )
        self._optimizer_inner_wall_weight = float(
            parameters.get("optimizer_inner_wall_weight", 2.0)
        )
        self._optimizer_inner_wall_top_weight = float(
            parameters.get("optimizer_inner_wall_top_weight", 1.0)
        )
        self._inner_wall_axis = str(
            parameters.get("inner_wall_axis", "x")
        ).strip().lower()
        self._inner_wall_distance_threshold_m = float(
            parameters.get("inner_wall_distance_threshold_m", 0.015)
        )
        self._inner_wall_ransac_iterations = int(
            parameters.get("inner_wall_ransac_iterations", 300)
        )
        self._inner_wall_min_inlier_ratio = float(
            parameters.get("inner_wall_min_inlier_ratio", 0.05)
        )
        self._inner_wall_max_normal_angle_deg = float(
            parameters.get("inner_wall_max_normal_angle_deg", 15.0)
        )
        self._inner_wall_far_quantile = float(
            parameters.get("inner_wall_far_quantile", 0.55)
        )
        self._inner_wall_top_quantile = float(
            parameters.get("inner_wall_top_quantile", 0.98)
        )
        self._inner_wall_min_vertical_span_ratio = float(
            parameters.get("inner_wall_min_vertical_span_ratio", 0.35)
        )
        self._optimizer_robust_scale_m = float(
            parameters["optimizer_robust_loss_scale_m"]
        )
        self._optimizer_max_iterations = int(parameters["optimizer_max_iterations"])
        self._optimizer_max_surface_rmse_m = float(
            parameters["optimizer_max_surface_rmse_m"]
        )
        self._optimizer_min_surface_inlier_ratio = float(
            parameters["optimizer_min_surface_inlier_ratio"]
        )
        self._optimizer_loss = str(parameters.get("optimizer_loss", "soft_l1"))
        self._output_base_limits = tuple(
            float(parameters.get(name, default))
            for name, default in (
                ("output_base_x_min_m", -2.0),
                ("output_base_x_max_m", 2.0),
                ("output_base_y_min_m", -2.0),
                ("output_base_y_max_m", 2.0),
                ("output_base_z_min_m", -0.2),
                ("output_base_z_max_m", 2.0),
            )
        )
        camera_to_base = np.asarray(
            self.camera_to_base_row_major, dtype=np.float64
        ).reshape(4, 4)
        self._rotation_camera_to_base = camera_to_base[:3, :3]
        self._translation_camera_to_base = camera_to_base[:3, 3]
        self._rotation_camera_from_box = self._rotation_camera_to_base.T
        expected_normal = self._rotation_camera_to_base.T @ np.array(
            [0.0, 0.0, 1.0], dtype=np.float64
        )
        expected_norm = float(np.linalg.norm(expected_normal))
        self._expected_support_normal_camera = expected_normal / expected_norm
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

    def process_point_cloud(
        self, point_cloud: CameraPointCloud
    ) -> PointCloudProcessingResult:
        """Run SOR, optional support-plane removal and 3D clustering."""
        filtered_cloud = self.remove_statistical_outliers(point_cloud)
        support_plane = self.fit_support_plane(filtered_cloud)
        candidate_cloud = filtered_cloud
        if support_plane is not None:
            candidate_cloud = filtered_cloud.subset(
                np.flatnonzero(~support_plane.inlier_mask)
            )

        if candidate_cloud.points_camera_m.shape[0] < self._min_cluster_points:
            support_plane = None
            candidate_cloud = filtered_cloud

        clusters = self.cluster_point_cloud(candidate_cloud)
        if support_plane is not None and not clusters:
            support_plane = None
            candidate_cloud = filtered_cloud
            clusters = self.cluster_point_cloud(candidate_cloud)
        if support_plane is not None and clusters:
            selected_cluster = clusters[0]
            selected_centroid = np.asarray(
                selected_cluster.centroid_camera_m, dtype=np.float64
            )
            plane_normal = np.asarray(support_plane.normal_camera, dtype=np.float64)
            support_gap = float(plane_normal @ selected_centroid + support_plane.offset)
            min_gap = max(0.02, 0.25 * self.box_size_m[2])
            max_gap = max(0.20, 2.0 * self.box_size_m[2])
            if not min_gap <= support_gap <= max_gap:
                support_plane = None
                candidate_cloud = filtered_cloud
                clusters = self.cluster_point_cloud(candidate_cloud)

        if not clusters:
            raise PointCloudProcessingError(
                "No 3D cluster met pointcloud_min_cluster_points"
            )

        selected_cluster = clusters[0]
        # Cluster indices refer to candidate_cloud, which may already exclude
        # support-plane points; keep the saved cloud aligned with the optimizer input.
        selected_cloud = candidate_cloud.subset(selected_cluster.indices)
        inner_wall_plane = self.fit_inner_wall(selected_cloud)
        return PointCloudProcessingResult(
            filtered_cloud=filtered_cloud,
            candidate_cloud=candidate_cloud,
            support_plane=support_plane,
            clusters=clusters,
            selected_cluster=selected_cluster,
            selected_cloud=selected_cloud,
            sor_removed_count=(
                point_cloud.points_camera_m.shape[0]
                - filtered_cloud.points_camera_m.shape[0]
            ),
            inner_wall_plane=inner_wall_plane,
        )

    def fit_support_plane(
        self, point_cloud: CameraPointCloud
    ) -> SupportPlane | None:
        """Fit an optional conveyor plane with RANSAC and a normal constraint."""
        points = np.asarray(point_cloud.points_camera_m, dtype=np.float64)
        if points.shape[0] < 3:
            return None

        rng = np.random.default_rng(0)
        best_mask: np.ndarray | None = None
        best_normal: np.ndarray | None = None
        best_offset = 0.0
        best_count = 0
        cos_limit = math.cos(math.radians(self._support_plane_max_angle_deg))
        min_inliers = max(
            3, int(math.ceil(self._support_plane_min_ratio * points.shape[0]))
        )

        for _ in range(self._support_plane_iterations):
            sample_indices = rng.choice(points.shape[0], size=3, replace=False)
            p1, p2, p3 = points[sample_indices]
            normal = np.cross(p2 - p1, p3 - p1)
            normal_norm = float(np.linalg.norm(normal))
            if normal_norm <= 1.0e-9:
                continue
            normal /= normal_norm
            alignment = float(abs(normal @ self._expected_support_normal_camera))
            if alignment < cos_limit:
                continue
            if float(normal @ self._expected_support_normal_camera) < 0.0:
                normal = -normal
            offset = -float(normal @ p1)
            distances = np.abs(points @ normal + offset)
            mask = distances <= self._support_plane_threshold_m
            count = int(np.count_nonzero(mask))
            if count > best_count:
                best_count = count
                best_mask = mask
                best_normal = normal
                best_offset = offset

        if best_mask is None or best_normal is None or best_count < min_inliers:
            return None

        inlier_points = points[best_mask]
        center = inlier_points.mean(axis=0)
        _, _, vh = np.linalg.svd(inlier_points - center, full_matrices=False)
        refined_normal = vh[-1]
        if float(refined_normal @ self._expected_support_normal_camera) < 0.0:
            refined_normal = -refined_normal
        refined_normal /= float(np.linalg.norm(refined_normal))
        refined_offset = -float(refined_normal @ center)
        refined_distances = np.abs(points @ refined_normal + refined_offset)
        refined_mask = refined_distances <= self._support_plane_threshold_m
        refined_count = int(np.count_nonzero(refined_mask))
        refined_ratio = refined_count / points.shape[0]
        angle_cos = float(
            np.clip(
                refined_normal @ self._expected_support_normal_camera,
                -1.0,
                1.0,
            )
        )
        angle_deg = math.degrees(math.acos(angle_cos))
        if refined_count < min_inliers or angle_deg > self._support_plane_max_angle_deg:
            return None
        return SupportPlane(
            normal_camera=tuple(float(value) for value in refined_normal),
            offset=refined_offset,
            inlier_mask=refined_mask,
            inlier_count=refined_count,
            inlier_ratio=float(refined_ratio),
            normal_angle_deg=float(angle_deg),
        )

    def fit_inner_wall(
        self, point_cloud: CameraPointCloud
    ) -> InnerWallPlane | None:
        """Fit the far inner wall using the nominal horizontal box axis."""
        points = np.asarray(point_cloud.points_camera_m, dtype=np.float64)
        if points.shape[0] < 3:
            return None

        axis_index = {"x": 0, "y": 1}.get(self._inner_wall_axis)
        if axis_index is None:
            return None
        wall_axis_camera = self._rotation_camera_from_box[:, axis_index]
        axis_norm = float(np.linalg.norm(wall_axis_camera))
        if axis_norm <= 1.0e-9:
            return None
        wall_axis_camera /= axis_norm
        far_sign = 1.0 if wall_axis_camera[2] >= 0.0 else -1.0
        expected_normal = far_sign * wall_axis_camera
        far_coordinate = points @ expected_normal
        far_threshold = float(
            np.quantile(far_coordinate, self._inner_wall_far_quantile)
        )
        far_indices = np.flatnonzero(far_coordinate >= far_threshold)
        if far_indices.size < 3:
            return None
        far_points = points[far_indices]

        rng = np.random.default_rng(1)
        cos_limit = math.cos(math.radians(self._inner_wall_max_normal_angle_deg))
        best_mask: np.ndarray | None = None
        best_normal: np.ndarray | None = None
        best_offset = 0.0
        best_count = 0
        for _ in range(self._inner_wall_ransac_iterations):
            sample_indices = rng.choice(far_points.shape[0], size=3, replace=False)
            p1, p2, p3 = far_points[sample_indices]
            normal = np.cross(p2 - p1, p3 - p1)
            normal_norm = float(np.linalg.norm(normal))
            if normal_norm <= 1.0e-9:
                continue
            normal /= normal_norm
            alignment = float(abs(normal @ expected_normal))
            if alignment < cos_limit:
                continue
            if float(normal @ expected_normal) < 0.0:
                normal = -normal
            offset = -float(normal @ p1)
            distances = np.abs(far_points @ normal + offset)
            mask = distances <= self._inner_wall_distance_threshold_m
            count = int(np.count_nonzero(mask))
            if count > best_count:
                best_count = count
                best_mask = mask
                best_normal = normal
                best_offset = offset

        min_inliers = max(
            3,
            int(math.ceil(self._inner_wall_min_inlier_ratio * points.shape[0])),
        )
        if best_mask is None or best_normal is None or best_count < min_inliers:
            return None

        inlier_points = far_points[best_mask]
        center = inlier_points.mean(axis=0)
        _, _, vh = np.linalg.svd(inlier_points - center, full_matrices=False)
        refined_normal = vh[-1]
        if float(refined_normal @ expected_normal) < 0.0:
            refined_normal = -refined_normal
        refined_normal /= float(np.linalg.norm(refined_normal))
        refined_offset = -float(refined_normal @ center)
        refined_distances = np.abs(points @ refined_normal + refined_offset)
        refined_mask = refined_distances <= self._inner_wall_distance_threshold_m
        refined_count = int(np.count_nonzero(refined_mask))
        refined_ratio = refined_count / points.shape[0]
        angle_cos = float(
            np.clip(refined_normal @ expected_normal, -1.0, 1.0)
        )
        angle_deg = math.degrees(math.acos(angle_cos))
        if (
            refined_count < min_inliers
            or angle_deg > self._inner_wall_max_normal_angle_deg
        ):
            return None
        # Keep the measured plane location, but express its equation with the
        # nominal fixed-pose normal. This prevents a small box tilt from being
        # misinterpreted as a large center shift along the coupled Y/Z axes.
        nominal_offset = -float(expected_normal @ points[refined_mask].mean(axis=0))
        nominal_z_axis = self._rotation_camera_from_box[:, 2]
        wall_height_coordinates = points[refined_mask] @ nominal_z_axis
        top_edge_span_m = float(
            wall_height_coordinates.max() - wall_height_coordinates.min()
        )
        min_vertical_span_m = (
            self._inner_wall_min_vertical_span_ratio * self.box_size_m[2]
        )
        top_edge_coordinate = None
        if top_edge_span_m >= min_vertical_span_m:
            top_edge_coordinate = float(
                np.quantile(
                    wall_height_coordinates,
                    self._inner_wall_top_quantile,
                )
            )
        return InnerWallPlane(
            normal_camera=tuple(float(value) for value in refined_normal),
            nominal_normal_camera=tuple(float(value) for value in expected_normal),
            offset=nominal_offset,
            inlier_mask=refined_mask,
            inlier_count=refined_count,
            inlier_ratio=float(refined_ratio),
            normal_angle_deg=float(angle_deg),
            local_axis_index=axis_index,
            top_edge_coordinate=top_edge_coordinate,
            top_edge_span_m=top_edge_span_m,
        )

    def remove_statistical_outliers(
        self, point_cloud: CameraPointCloud
    ) -> CameraPointCloud:
        """Remove points whose mean k-neighbor distance is statistically large."""
        points = np.asarray(point_cloud.points_camera_m, dtype=np.float64)
        point_count = points.shape[0]
        if point_count <= self._sor_neighbors:
            return point_cloud

        tree = self._make_tree(points)
        distances, _ = tree.query(
            points,
            k=min(point_count, self._sor_neighbors + 1),
        )
        mean_neighbor_distance = np.asarray(distances[:, 1:], dtype=np.float64).mean(
            axis=1
        )
        mean_distance = float(np.mean(mean_neighbor_distance))
        std_distance = float(np.std(mean_neighbor_distance))
        threshold = mean_distance + self._sor_std_ratio * std_distance
        keep = mean_neighbor_distance <= threshold

        kept_count = int(np.count_nonzero(keep))
        if kept_count < self._min_cluster_points:
            raise PointCloudProcessingError(
                "Statistical outlier removal rejected too many points: "
                f"{kept_count} remain"
            )
        return point_cloud.subset(np.flatnonzero(keep))

    def cluster_point_cloud(
        self, point_cloud: CameraPointCloud
    ) -> tuple[PointCloudCluster, ...]:
        """Find connected components using a 3D Euclidean neighborhood."""
        points = np.asarray(point_cloud.points_camera_m, dtype=np.float64)
        tree = self._make_tree(points)
        visited = np.zeros(points.shape[0], dtype=bool)
        raw_clusters: list[np.ndarray] = []

        for seed in range(points.shape[0]):
            if visited[seed]:
                continue
            queue = [seed]
            visited[seed] = True
            component: list[int] = []
            while queue:
                current = queue.pop()
                component.append(current)
                neighbors = tree.query_ball_point(
                    points[current], self._cluster_tolerance_m
                )
                for neighbor in neighbors:
                    if not visited[neighbor]:
                        visited[neighbor] = True
                        queue.append(neighbor)

            if len(component) >= self._min_cluster_points:
                raw_clusters.append(np.asarray(component, dtype=np.int64))

        if not raw_clusters:
            return ()

        max_count = max(indices.size for indices in raw_clusters)
        roi_width = max(1, point_cloud.roi_xyxy[2] - point_cloud.roi_xyxy[0])
        roi_height = max(1, point_cloud.roi_xyxy[3] - point_cloud.roi_xyxy[1])
        clusters: list[PointCloudCluster] = []
        for indices in raw_clusters:
            cluster_points = points[indices]
            cluster_pixels = point_cloud.pixels_uv[indices]
            depths = point_cloud.depths_m[indices]
            u_min, v_min = np.floor(cluster_pixels.min(axis=0)).astype(int)
            u_max, v_max = np.ceil(cluster_pixels.max(axis=0)).astype(int)
            coverage = np.sqrt(
                max(1, u_max - u_min) / roi_width
                * max(1, v_max - v_min) / roi_height
            )
            depth_range = float(np.ptp(depths))
            depth_consistency = 1.0 / (
                1.0 + depth_range / max(0.02, float(np.median(depths)) * 0.05)
            )
            count_score = indices.size / max_count
            score = float(
                0.65 * count_score
                + 0.20 * min(1.0, coverage)
                + 0.15 * depth_consistency
            )
            clusters.append(
                PointCloudCluster(
                    indices=indices,
                    centroid_camera_m=tuple(
                        float(value) for value in cluster_points.mean(axis=0)
                    ),
                    point_count=int(indices.size),
                    pixel_xyxy=(u_min, v_min, u_max, v_max),
                    depth_min_m=float(depths.min()),
                    depth_median_m=float(np.median(depths)),
                    depth_max_m=float(depths.max()),
                    score=score,
                )
            )
        return tuple(sorted(clusters, key=lambda cluster: cluster.score, reverse=True))

    def estimate(self, frame: Any, detection: Any) -> PositionEstimate:
        """Extract, process and optimize one detection."""
        point_cloud = self.extract_point_cloud(frame, detection)
        processing = self.process_point_cloud(point_cloud)
        return self.estimate_from_processing(frame, detection, processing)

    def estimate_from_processing(
        self,
        frame: Any,
        detection: Any,
        processing: PointCloudProcessingResult,
    ) -> PositionEstimate:
        """Optimize a center from an already processed point cloud."""
        if least_squares is None:
            raise PositionEstimationError(
                "scipy is required for fixed-size box position optimization"
            )

        points = np.asarray(
            processing.selected_cloud.points_camera_m, dtype=np.float64
        )
        if points.shape[0] < self._min_cluster_points:
            raise PositionEstimationError(
                f"Selected candidate has too few points: {points.shape[0]}"
            )

        intrinsics = frame.color_intrinsics
        camera_intrinsics = (
            float(intrinsics.fx),
            float(intrinsics.fy),
            float(intrinsics.ppx),
            float(intrinsics.ppy),
        )
        image_height, image_width = frame.color_bgr.shape[:2]
        bbox = np.array(
            [detection.x_min, detection.y_min, detection.x_max, detection.y_max],
            dtype=np.float64,
        )
        bounds = self._optimization_bounds(points)
        seeds = self._initial_center_candidates(
            points,
            bbox,
            camera_intrinsics,
            processing.support_plane,
            processing.inner_wall_plane,
        )
        best_result: Any | None = None
        best_cost = float("inf")
        for seed in seeds:
            x0 = np.clip(seed, bounds[0] + 1.0e-6, bounds[1] - 1.0e-6)
            result = least_squares(
                self._optimization_residuals,
                x0,
                args=(
                    points,
                    bbox,
                    camera_intrinsics,
                    (image_width, image_height),
                    processing.support_plane,
                    processing.inner_wall_plane,
                ),
                bounds=bounds,
                loss=self._optimizer_loss,
                f_scale=1.0,
                max_nfev=self._optimizer_max_iterations,
            )
            if np.isfinite(result.cost) and result.cost < best_cost:
                best_result = result
                best_cost = float(result.cost)

        if best_result is None or not np.all(np.isfinite(best_result.x)):
            raise PositionEstimationError("Position optimizer returned no finite result")

        center_camera = np.asarray(best_result.x, dtype=np.float64)
        metrics = self._quality_metrics(
            center_camera,
            points,
            bbox,
            camera_intrinsics,
            (image_width, image_height),
        )
        center_base = (
            self._rotation_camera_to_base @ center_camera
            + self._translation_camera_to_base
        )
        if not self._inside_output_base_limits(center_base):
            raise PositionEstimationError(
                f"Estimated base_link center is outside configured limits: {center_base}"
            )
        if center_camera[2] <= 0.0:
            raise PositionEstimationError("Estimated camera depth is not positive")
        if not bool(best_result.success):
            raise PositionEstimationError(
                f"Position optimizer did not converge: {best_result.message}"
            )
        if metrics["surface_rmse_m"] > self._optimizer_max_surface_rmse_m:
            raise PositionEstimationError(
                "Surface residual is too large: "
                f"{metrics['surface_rmse_m']:.4f} m"
            )
        if metrics["surface_inlier_ratio"] < self._optimizer_min_surface_inlier_ratio:
            raise PositionEstimationError(
                "Too few candidate points agree with the fixed-size model: "
                f"{metrics['surface_inlier_ratio']:.3f}"
            )
        if metrics["bbox_iou"] < 0.05:
            raise PositionEstimationError(
                f"Projected fixed-size model does not overlap YOLO bbox: IoU={metrics['bbox_iou']:.3f}"
            )

        confidence = min(
            1.0,
            metrics["surface_inlier_ratio"]
            * math.exp(
                -metrics["surface_rmse_m"]
                / max(1.0e-6, self._optimizer_robust_scale_m)
            ),
        )
        if processing.support_plane is None:
            confidence *= 0.80
        confidence *= min(1.0, max(0.0, metrics["bbox_iou"] * 2.0))
        return PositionEstimate(
            center_camera_m=tuple(float(value) for value in center_camera),
            center_base_m=tuple(float(value) for value in center_base),
            confidence=float(confidence),
            point_count=int(points.shape[0]),
            support_plane_valid=processing.support_plane is not None,
            surface_rmse_m=metrics["surface_rmse_m"],
            surface_inlier_ratio=metrics["surface_inlier_ratio"],
            inner_wall_valid=processing.inner_wall_plane is not None,
        )

    def _initial_center_candidates(
        self,
        points: np.ndarray,
        bbox: np.ndarray,
        intrinsics: tuple[float, float, float, float],
        support_plane: SupportPlane | None,
        inner_wall_plane: InnerWallPlane | None,
    ) -> tuple[np.ndarray, ...]:
        fx, fy, cx, cy = intrinsics
        u_center = 0.5 * (bbox[0] + bbox[2])
        v_center = 0.5 * (bbox[1] + bbox[3])
        depth_candidates = (
            float(np.percentile(points[:, 2], 35.0)),
            float(np.median(points[:, 2])),
            float(np.percentile(points[:, 2], 65.0)),
        )
        seeds = [np.median(points, axis=0)]
        for depth in depth_candidates:
            seeds.append(
                np.array(
                    [
                        (u_center - cx) * depth / fx,
                        (v_center - cy) * depth / fy,
                        depth,
                    ],
                    dtype=np.float64,
                )
            )

        if support_plane is not None:
            normal = np.asarray(support_plane.normal_camera, dtype=np.float64)
            box_z_axis = self._rotation_camera_from_box @ np.array(
                [0.0, 0.0, 1.0], dtype=np.float64
            )
            for seed in tuple(seeds):
                correction = -(
                    float(normal @ (seed - self.box_size_m[2] * 0.5 * box_z_axis))
                    + support_plane.offset
                )
                seeds.append(seed + correction * normal)

        if inner_wall_plane is not None:
            normal = np.asarray(
                inner_wall_plane.nominal_normal_camera, dtype=np.float64
            )
            wall_half_size = 0.5 * self.box_size_m[inner_wall_plane.local_axis_index]
            for seed in tuple(seeds):
                # The fitted plane is the far face, so n^T C + d + h = 0.
                correction = -(
                    float(normal @ seed)
                    + inner_wall_plane.offset
                    + wall_half_size
                )
                seeds.append(seed + correction * normal)
            if inner_wall_plane.top_edge_coordinate is not None:
                box_z_axis = self._rotation_camera_from_box[:, 2]
                for seed in tuple(seeds):
                    correction = -float(
                        box_z_axis @ seed
                        + 0.5 * self.box_size_m[2]
                        - inner_wall_plane.top_edge_coordinate
                    )
                    seeds.append(seed + correction * box_z_axis)
        return tuple(seeds)

    def _optimization_bounds(
        self, points: np.ndarray
    ) -> tuple[np.ndarray, np.ndarray]:
        margin = max(0.20, 2.0 * max(self.box_size_m))
        lower = points.min(axis=0) - margin
        upper = points.max(axis=0) + margin
        half_size = 0.5 * np.asarray(self.box_size_m, dtype=np.float64)
        signs = np.array(
            [
                [-1.0, -1.0, -1.0],
                [-1.0, -1.0, 1.0],
                [-1.0, 1.0, -1.0],
                [-1.0, 1.0, 1.0],
                [1.0, -1.0, -1.0],
                [1.0, -1.0, 1.0],
                [1.0, 1.0, -1.0],
                [1.0, 1.0, 1.0],
            ],
            dtype=np.float64,
        )
        box_corner_offsets_camera = (
            signs * half_size
        ) @ self._rotation_camera_from_box.T
        camera_z_extent = float(np.max(np.abs(box_corner_offsets_camera[:, 2])))
        # The center only needs to keep every model corner in front of the
        # camera; it must not be forced to depth_min + max(box_size).
        lower[2] = max(
            0.05 + camera_z_extent,
            points[:, 2].min() - camera_z_extent,
        )
        upper[2] = max(
            upper[2],
            points[:, 2].max() + camera_z_extent,
            self._depth_max_m + camera_z_extent,
        )
        return lower, upper

    def _optimization_residuals(
        self,
        center_camera: np.ndarray,
        points: np.ndarray,
        bbox: np.ndarray,
        intrinsics: tuple[float, float, float, float],
        image_size: tuple[int, int],
        support_plane: SupportPlane | None,
        inner_wall_plane: InnerWallPlane | None,
    ) -> np.ndarray:
        q = self._box_local_points(points, center_camera)
        half_size = 0.5 * np.asarray(self.box_size_m, dtype=np.float64)
        outside = np.maximum(np.abs(q) - half_size, 0.0)
        containment = np.linalg.norm(outside, axis=1)

        distance_to_face = np.min(
            np.abs(half_size[None, :] - np.abs(q)), axis=1
        )
        surface_band = max(0.010, 1.5 * self._optimizer_robust_scale_m)
        surface = np.maximum(distance_to_face - surface_band, 0.0)
        residuals = [
            math.sqrt(self._optimizer_containment_weight)
            * containment
            / self._optimizer_robust_scale_m,
            math.sqrt(self._optimizer_surface_weight)
            * surface
            / self._optimizer_robust_scale_m,
        ]

        if support_plane is not None:
            normal = np.asarray(support_plane.normal_camera, dtype=np.float64)
            box_z_axis = self._rotation_camera_from_box @ np.array(
                [0.0, 0.0, 1.0], dtype=np.float64
            )
            bottom_center = center_camera - half_size[2] * box_z_axis
            support_residual = float(normal @ bottom_center + support_plane.offset)
            residuals.append(
                np.array(
                    [
                        math.sqrt(self._optimizer_support_weight)
                        * support_residual
                        / self._optimizer_robust_scale_m
                    ],
                    dtype=np.float64,
                )
            )

        if inner_wall_plane is not None:
            normal = np.asarray(
                inner_wall_plane.nominal_normal_camera, dtype=np.float64
            )
            wall_half_size = 0.5 * self.box_size_m[inner_wall_plane.local_axis_index]
            wall_residual = float(
                normal @ center_camera
                + inner_wall_plane.offset
                + wall_half_size
            )
            residuals.append(
                np.array(
                    [
                        math.sqrt(self._optimizer_inner_wall_weight)
                        * wall_residual
                        / self._optimizer_robust_scale_m
                    ],
                    dtype=np.float64,
                )
            )
            if inner_wall_plane.top_edge_coordinate is not None:
                box_z_axis = self._rotation_camera_from_box[:, 2]
                top_residual = float(
                    box_z_axis @ center_camera
                    + half_size[2]
                    - inner_wall_plane.top_edge_coordinate
                )
                residuals.append(
                    np.array(
                        [
                            math.sqrt(self._optimizer_inner_wall_top_weight)
                            * top_residual
                            / self._optimizer_robust_scale_m
                        ],
                        dtype=np.float64,
                    )
                )

        model_bbox = self._projected_model_bbox(
            center_camera, intrinsics, image_size
        )
        bbox_scale_px = max(20.0, 0.05 * max(image_size))
        bbox_cover = np.maximum(
            np.array(
                [
                    model_bbox[0] - bbox[0],
                    model_bbox[1] - bbox[1],
                    bbox[2] - model_bbox[2],
                    bbox[3] - model_bbox[3],
                ],
                dtype=np.float64,
            ),
            0.0,
        )
        bbox_fit = self._bbox_visibility_weights(bbox, image_size) * (
            model_bbox - bbox
        )
        residuals.append(
            math.sqrt(self._optimizer_bbox_weight)
            * (bbox_cover + bbox_fit)
            / bbox_scale_px
        )
        return np.concatenate(residuals)

    def _quality_metrics(
        self,
        center_camera: np.ndarray,
        points: np.ndarray,
        bbox: np.ndarray,
        intrinsics: tuple[float, float, float, float],
        image_size: tuple[int, int],
    ) -> dict[str, float]:
        q = self._box_local_points(points, center_camera)
        half_size = 0.5 * np.asarray(self.box_size_m, dtype=np.float64)
        distance_to_face = np.min(
            np.abs(half_size[None, :] - np.abs(q)), axis=1
        )
        surface_band = max(0.010, 1.5 * self._optimizer_robust_scale_m)
        surface_residual = np.maximum(distance_to_face - surface_band, 0.0)
        outside = np.maximum(np.abs(q) - half_size, 0.0)
        inlier_mask = (surface_residual <= surface_band) & (
            np.linalg.norm(outside, axis=1) <= self._optimizer_robust_scale_m
        )
        model_bbox = self._projected_model_bbox(
            center_camera, intrinsics, image_size
        )
        visible_model_bbox = self._clip_bbox_to_image(model_bbox, image_size)
        return {
            "surface_rmse_m": float(np.sqrt(np.mean(surface_residual**2))),
            "surface_inlier_ratio": float(np.mean(inlier_mask)),
            "bbox_iou": self._bbox_iou(visible_model_bbox, bbox),
        }

    def _bbox_visibility_weights(
        self,
        bbox: np.ndarray,
        image_size: tuple[int, int],
    ) -> np.ndarray:
        """Reduce equality matching near image borders without hard branching."""
        image_width, image_height = image_size
        transition = self._optimizer_bbox_clip_transition_px
        return np.clip(
            np.array(
                [
                    bbox[0],
                    bbox[1],
                    image_width - bbox[2],
                    image_height - bbox[3],
                ],
                dtype=np.float64,
            )
            / transition,
            0.0,
            1.0,
        )

    @staticmethod
    def _clip_bbox_to_image(
        bbox: np.ndarray,
        image_size: tuple[int, int],
    ) -> np.ndarray:
        image_width, image_height = image_size
        clipped = np.asarray(bbox, dtype=np.float64).copy()
        clipped[[0, 2]] = np.clip(clipped[[0, 2]], 0.0, float(image_width))
        clipped[[1, 3]] = np.clip(clipped[[1, 3]], 0.0, float(image_height))
        if clipped[2] < clipped[0] or clipped[3] < clipped[1]:
            return np.zeros(4, dtype=np.float64)
        return clipped

    def _box_local_points(
        self, points: np.ndarray, center_camera: np.ndarray
    ) -> np.ndarray:
        return (points - center_camera) @ self._rotation_camera_from_box

    def _projected_model_bbox(
        self,
        center_camera: np.ndarray,
        intrinsics: tuple[float, float, float, float],
        image_size: tuple[int, int],
    ) -> np.ndarray:
        fx, fy, cx, cy = intrinsics
        half_size = 0.5 * np.asarray(self.box_size_m, dtype=np.float64)
        signs = np.array(
            [
                [-1.0, -1.0, -1.0],
                [-1.0, -1.0, 1.0],
                [-1.0, 1.0, -1.0],
                [-1.0, 1.0, 1.0],
                [1.0, -1.0, -1.0],
                [1.0, -1.0, 1.0],
                [1.0, 1.0, -1.0],
                [1.0, 1.0, 1.0],
            ],
            dtype=np.float64,
        )
        corners_camera = center_camera + (
            signs * half_size
        ) @ self._rotation_camera_from_box.T
        if np.any(corners_camera[:, 2] <= 1.0e-6):
            return np.array(
                [0.0, 0.0, float(image_size[0]), float(image_size[1])],
                dtype=np.float64,
            )
        u = fx * corners_camera[:, 0] / corners_camera[:, 2] + cx
        v = fy * corners_camera[:, 1] / corners_camera[:, 2] + cy
        return np.array([u.min(), v.min(), u.max(), v.max()], dtype=np.float64)

    @staticmethod
    def _bbox_iou(first: np.ndarray, second: np.ndarray) -> float:
        left = max(first[0], second[0])
        top = max(first[1], second[1])
        right = min(first[2], second[2])
        bottom = min(first[3], second[3])
        intersection = max(0.0, right - left) * max(0.0, bottom - top)
        first_area = max(0.0, first[2] - first[0]) * max(0.0, first[3] - first[1])
        second_area = max(0.0, second[2] - second[0]) * max(0.0, second[3] - second[1])
        union = first_area + second_area - intersection
        return float(intersection / union) if union > 1.0e-9 else 0.0

    def _inside_output_base_limits(
        self, center_base: np.ndarray
    ) -> bool:
        x_min, x_max, y_min, y_max, z_min, z_max = self._output_base_limits
        return bool(
            x_min <= center_base[0] <= x_max
            and y_min <= center_base[1] <= y_max
            and z_min <= center_base[2] <= z_max
        )

    @staticmethod
    def _make_tree(points: np.ndarray) -> Any:
        if cKDTree is None:
            raise PointCloudProcessingError(
                "scipy is required for point-cloud SOR and 3D clustering"
            )
        return cKDTree(points)

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
        if self._sor_neighbors <= 0:
            raise ValueError("pointcloud_statistical_neighbors must be positive")
        if not np.isfinite(self._sor_std_ratio) or self._sor_std_ratio < 0.0:
            raise ValueError(
                "pointcloud_statistical_std_ratio must be finite and non-negative"
            )
        if (
            not np.isfinite(self._cluster_tolerance_m)
            or self._cluster_tolerance_m <= 0.0
        ):
            raise ValueError("pointcloud_cluster_tolerance_m must be positive")
        if self._min_cluster_points <= 0:
            raise ValueError("pointcloud_min_cluster_points must be positive")
        if (
            not np.isfinite(self._support_plane_threshold_m)
            or self._support_plane_threshold_m <= 0.0
        ):
            raise ValueError("support_plane_distance_threshold_m must be positive")
        if self._support_plane_iterations <= 0:
            raise ValueError("support_plane_ransac_iterations must be positive")
        if (
            not np.isfinite(self._support_plane_min_ratio)
            or not 0.0 < self._support_plane_min_ratio <= 1.0
        ):
            raise ValueError(
                "support_plane_min_inlier_ratio must be in the interval (0, 1]"
            )
        if (
            not np.isfinite(self._support_plane_max_angle_deg)
            or not 0.0 <= self._support_plane_max_angle_deg <= 90.0
        ):
            raise ValueError(
                "support_plane_max_normal_angle_deg must be in [0, 90]"
            )
        optimizer_weights = (
            self._optimizer_surface_weight,
            self._optimizer_containment_weight,
            self._optimizer_support_weight,
            self._optimizer_bbox_weight,
            self._optimizer_inner_wall_weight,
            self._optimizer_inner_wall_top_weight,
        )
        if not all(np.isfinite(value) and value >= 0.0 for value in optimizer_weights):
            raise ValueError("optimizer weights must be finite and non-negative")
        if (
            not np.isfinite(self._optimizer_robust_scale_m)
            or self._optimizer_robust_scale_m <= 0.0
        ):
            raise ValueError("optimizer_robust_loss_scale_m must be positive")
        if self._optimizer_max_iterations <= 0:
            raise ValueError("optimizer_max_iterations must be positive")
        if (
            not np.isfinite(self._optimizer_max_surface_rmse_m)
            or self._optimizer_max_surface_rmse_m <= 0.0
        ):
            raise ValueError("optimizer_max_surface_rmse_m must be positive")
        if (
            not np.isfinite(self._optimizer_min_surface_inlier_ratio)
            or not 0.0 <= self._optimizer_min_surface_inlier_ratio <= 1.0
        ):
            raise ValueError(
                "optimizer_min_surface_inlier_ratio must be in [0, 1]"
            )
        if (
            not np.isfinite(self._optimizer_bbox_clip_transition_px)
            or self._optimizer_bbox_clip_transition_px <= 0.0
        ):
            raise ValueError("optimizer_bbox_clip_transition_px must be positive")
        if self._inner_wall_axis not in {"x", "y"}:
            raise ValueError("inner_wall_axis must be x or y")
        if (
            not np.isfinite(self._inner_wall_distance_threshold_m)
            or self._inner_wall_distance_threshold_m <= 0.0
        ):
            raise ValueError("inner_wall_distance_threshold_m must be positive")
        if self._inner_wall_ransac_iterations <= 0:
            raise ValueError("inner_wall_ransac_iterations must be positive")
        if (
            not np.isfinite(self._inner_wall_min_inlier_ratio)
            or not 0.0 < self._inner_wall_min_inlier_ratio <= 1.0
        ):
            raise ValueError("inner_wall_min_inlier_ratio must be in (0, 1]")
        if (
            not np.isfinite(self._inner_wall_max_normal_angle_deg)
            or not 0.0 <= self._inner_wall_max_normal_angle_deg <= 90.0
        ):
            raise ValueError("inner_wall_max_normal_angle_deg must be in [0, 90]")
        if (
            not np.isfinite(self._inner_wall_far_quantile)
            or not 0.0 < self._inner_wall_far_quantile < 1.0
        ):
            raise ValueError("inner_wall_far_quantile must be in (0, 1)")
        if (
            not np.isfinite(self._inner_wall_top_quantile)
            or not 0.5 < self._inner_wall_top_quantile < 1.0
        ):
            raise ValueError("inner_wall_top_quantile must be in (0.5, 1)")
        if (
            not np.isfinite(self._inner_wall_min_vertical_span_ratio)
            or not 0.0 < self._inner_wall_min_vertical_span_ratio <= 1.0
        ):
            raise ValueError(
                "inner_wall_min_vertical_span_ratio must be in (0, 1]"
            )
        if self._optimizer_loss not in {"soft_l1", "huber"}:
            raise ValueError("optimizer_loss must be soft_l1 or huber")
        if not all(
            np.isfinite(value)
            for value in self._output_base_limits
        ):
            raise ValueError("output_base limits must be finite")
