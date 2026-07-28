# HANDOFF

## 当前任务

工作区：`/home/ub/project/detect_box_ws`，ROS 2 Humble。

`detect_pkg/detect_server_node` 已切换为 D435 深度运动检测方案。节点收到 `/detect` 服务请求后采集短时间深度窗口，根据间隔帧深度差分生成运动候选，再从当前完整深度图中估计 box 当前观察位姿。

保留不变：

- 运行入口：`ros2 run detect_pkg detect_server_node`
- 服务：`/detect`
- 服务类型：`upper_limb_interface/srv/DetectAprilTag`
- 响应字段：`success/message/box_pose`
- 可视化话题：`box_pose`
- 外参语义：`T_base_box = T_base_camera * T_camera_box`
- 默认 `camera_to_base_row_major` 值沿用原真机低腰外参

## 主要文件

- `src/detect_pkg/src/detect_server_node.cpp`
- `src/detect_pkg/include/detect_pkg/depth_box_detection.hpp`
- `src/detect_pkg/config/depth_box_detection.yaml`
- `src/detect_pkg/CMakeLists.txt`
- `recognition_localization_flow.md`

## 当前实现链路

```text
DetectOnce
  -> D435 深度短窗口采集
  -> 间隔帧深度差分
  -> 多组差分投票
  -> 候选框右向扩张
  -> 当前深度图点云
  -> camera 坐标系 3D ROI
  -> 上沿候选平面 RANSAC
  -> 局部平面 PCA 估计观察位姿
  -> T_base_camera 转 base_link
  -> 返回 box_pose / 发布 Marker
```

## 启动

```bash
cd /home/ub/project/detect_box_ws
source install/setup.bash
ros2 run detect_pkg detect_server_node --ros-args \
  --params-file src/detect_pkg/config/depth_box_detection.yaml
```

测试客户端仍可使用：

```bash
ros2 run detect_pkg detect_client_node
```

## 边界

- 当前输出是运动中观察位姿，不是最终抓取位姿。
- `success=true` 只表示视觉产生了当前观察位姿。
- 失败时 `box_pose` 返回 `fallback_pose`。
- 需要现场实测箱体尺寸并调整 `box_outer_length_m/box_outer_width_m/box_outer_height_m`。
- 需要现场调 `roi_*`、`depth_difference_threshold_m`、`min_motion_votes`、候选框扩张像素。

## 验证建议

1. 空传送带请求，确认 `success=false` 且 `debug_depth/motion_failed_*.png` 没有大面积误检。
2. 箱体从右向左进入视野，确认候选框主要覆盖箱体已进入部分和右侧扩张区域。
3. 箱体只露出左侧小部分时，应优先返回部分进入或失败，不应输出伪完整中心。
4. 箱体大部分进入视野后，确认 `box_pose` 方向和位置在 `base_link` 下合理。
5. 根据调试图逐步收紧 3D ROI 和运动阈值。
