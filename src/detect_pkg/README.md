# detect_pkg

`detect_pkg` 是 D435 传送带开口箱检测服务包。当前实现不使用人工标记，收到服务请求后直接通过 RealSense 深度流采集一个短时间窗口，利用间隔帧深度差分生成运动候选，再从当前完整深度图中估计箱体当前观察位姿。

本阶段输出只用于观察、显示、记录和提示，不表示最终抓取位姿有效。

## 保留接口

运行入口保持不变：

```bash
ros2 run detect_pkg detect_server_node
```

服务接口保持不变：

```text
service: /detect
type: upper_limb_interface/srv/DetectAprilTag
request: bool capture_once
response: bool success, string message, geometry_msgs/Pose box_pose
```

话题接口保持不变：

```text
box_pose
```

`box_pose` 仍发布 `visualization_msgs/msg/Marker`，默认坐标系为 `base_link`。

## 检测流程

```text
DetectOnce 请求
  -> D435 深度短窗口采集，默认 0.8 s
  -> 间隔帧深度差分
  -> 多组差分投票
  -> 左前沿米制参考位置速度拟合
  -> 自右向左运动候选框右向扩张
  -> 最后一帧完整深度图候选点云
  -> camera 坐标系 3D ROI 过滤
  -> 上沿候选点平面 RANSAC
  -> 局部二维米制平面边线 RANSAC
  -> 凹槽验证和内侧方向粗判
  -> 垂直边角点结合实测长宽推算观察中心
  -> T_base_camera 外参转换到 base_link
```

失败时返回 `fallback_pose`。成功时 `box_pose` 是当前观察位姿，不应直接用于抓取。

## 外参

代码和参数中的外参语义保持为：

```text
T_base_box = T_base_camera * T_camera_box
```

默认 `camera_to_base_row_major` 沿用原真机低腰外参：

```text
[0.0000, -0.342020, 0.939693, 0.12972,
 -1.000000, 0.000000, 0.000000, 0.03250,
  0.000000, -0.939693, -0.342020, 0.24561,
  0.000000, 0.000000, 0.000000, 1.000000]
```

## 参数文件

默认参数位于：

```text
src/detect_pkg/config/depth_box_detection.yaml
```

启动示例：

```bash
ros2 run detect_pkg detect_server_node --ros-args \
  --params-file src/detect_pkg/config/depth_box_detection.yaml
```

主要参数：

```text
capture_window_sec: 0.8
depth_width/depth_height/depth_fps: 640 / 480 / 30
motion_frame_gap: 5
min_motion_votes: 3
depth_difference_threshold_m: 0.025
motion_min_valid_observations: 6
moving_velocity_threshold_mps / static_velocity_threshold_mps
motion_max_fit_rmse_m / motion_min_confidence
candidate_expand_right_px: 250
roi_x/y/z_min/max_m
line_inlier_distance_m: 0.012
line_min_inliers: 25
line_min_visible_length_m: 0.100
maximum_line_count: 4
parallel_tolerance_deg / perpendicular_tolerance_deg
cavity_min_drop_m: 0.040
cavity_min_score_partial / cavity_min_score_confirmed
cavity_min_inner_points_partial / cavity_min_inner_points_confirmed
cavity_side_min_offset_m / cavity_side_band_m
box_outer_length_m / box_outer_width_m / box_outer_height_m
fallback_pose
camera_to_base_row_major
```

调试图默认保存到：

```text
debug_depth/
```

其中 `motion_*.png` 是运动掩膜，`depth_*.png` 是最后一帧深度伪彩色图和候选框。

服务响应的 `message` 会包含 `velocity_mps`、`motion_span_m`、`velocity_rmse_m`、`lines`、`cavity_score`、`cavity_verified`、`center_from_dimensions` 等字段：

- `velocity_mps`：运动候选左前沿在 `camera` X 方向上的拟合速度，右到左通常为负值。
- `motion_span_m`：短窗口内前沿参考位置跨度。
- `velocity_rmse_m`：前沿位置线性拟合 RMSE。
- `lines`：当前上沿局部平面中拟合到的二维边线数量。
- `cavity_score`：候选内侧区域里，下凹点占有效点的比例。
- `cavity_verified`：凹槽分数和点数是否达到部分确认阈值。
- `center_from_dimensions`：是否使用垂直边角点和实测箱体长宽推算完整观察中心。

## 代码结构

```text
src/detect_pkg/src/detect_server_node.cpp
  ROS2 service、参数加载、Marker 发布
src/detect_pkg/src/d435_depth_frame_provider.cpp
  RealSense 深度短窗口采集
src/detect_pkg/src/depth_box_detector.cpp
  运动检测、运动估计、点云几何、位姿估计
```

## 编译

```bash
colcon build --packages-select detect_pkg --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
source install/setup.bash
```

如果接口包尚未构建，需要先构建或 source 已安装的 `upper_limb_interface`。
