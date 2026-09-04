# detect_pkg_sim

`detect_pkg_sim` 是实体检测包 `detect_pkg` 的仿真副本。它保留 YOLO-World、点云处理、固定尺寸箱体中心估计、`/detect` 服务类型和输出结构，只把 RGB-D 数据来源替换为 ROS 2 话题。

本包不打开 RealSense，不依赖 `pyrealsense2`，仅用于 x86 仿真环境。

## 输入与接口

默认订阅：

| 内容 | 话题 | 格式 |
|---|---|---|
| 彩色图 | `/camera/color/image_raw` | `sensor_msgs/Image`，`rgb8` |
| 彩色相机内参 | `/camera/color/camera_info` | `sensor_msgs/CameraInfo` |
| 对齐深度图 | `/camera/aligned_depth_to_color/image_raw` | `sensor_msgs/Image`，`32FC1`，单位 m |

相机采用标准光学坐标系：X 向右、Y 向下、Z 向前。

服务保持为：

```text
名称：/detect
类型：upper_limb_interface/srv/DetectAprilTag
请求：capture_once=true
响应：success、message、box_pose
```

收到请求后，节点等待一组请求之后发布的新同步 RGB-D 数据，不使用请求前缓存的旧帧。三个消息通过近似时间同步组成一组；超时后服务返回失败。

## 主要文件

```text
detect_pkg_sim/
├── config/box_position_estimation_sim.yaml
├── launch/box_position_estimation_sim.launch.py
├── detect_box_sim_pipeline/
│   ├── detect_server_node.py
│   ├── topic_rgbd_camera.py
│   ├── yolo_world_detector.py
│   ├── box_position_estimator.py
│   └── debug_snapshot.py
├── scripts/detect_server_node
├── CMakeLists.txt
└── package.xml
```

`topic_rgbd_camera.py` 负责：

- 同步 RGB、深度和 CameraInfo；
- 将 `rgb8` 转换为 YOLO 所需的 BGR 图像；
- 保持 `32FC1` 深度单位为米，深度比例固定为 `1.0`；
- 校验图像尺寸、编码和相机内参；
- 为每次服务请求等待一组新帧。

服务节点使用 `MultiThreadedExecutor`。订阅回调与服务回调属于不同 callback group，因此服务等待新帧时不会阻塞相机话题接收。

## 参数

默认参数文件：

```text
src/detect_pkg_sim/config/box_position_estimation_sim.yaml
```

话题输入相关参数：

```yaml
color_topic: /camera/color/image_raw
camera_info_topic: /camera/color/camera_info
aligned_depth_topic: /camera/aligned_depth_to_color/image_raw
rgbd_sync_queue_size: 20
rgbd_sync_tolerance_sec: 0.03
rgbd_wait_timeout_sec: 1.0
```

`camera_to_base_row_major` 必须表示仿真相机光学坐标系到仿真 `base_link` 的变换。当前初始值继承自实体包，接入具体仿真场景时必须核对。

## 本地构建

```bash
cd /home/ub/project/detect_box_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select upper_limb_interface detect_pkg_sim --symlink-install
source install/setup.bash
```

## 本地启动

确保检测 Python 环境中已经安装 PyTorch、Ultralytics、CLIP、NumPy、SciPy 和 OpenCV：

```bash
cd /home/ub/project/detect_box_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export DETECT_BOX_PYTHON=/home/ub/miniconda3/envs/py310/bin/python
ros2 launch detect_pkg_sim box_position_estimation_sim.launch.py
```

另一个终端调用：

```bash
source /opt/ros/humble/setup.bash
source /home/ub/project/detect_box_ws/install/setup.bash
ros2 service call /detect upper_limb_interface/srv/DetectAprilTag \
  "{capture_once: true}"
```

## 运行约束

- 仿真相机必须持续发布三个输入话题；
- 三个消息必须带有可同步的 `header.stamp`；
- 深度必须已经对齐到彩色图；
- `CameraInfo.k` 必须对应彩色图内参；
- 同一 ROS Domain 中不要同时启动两个同名 `/detect` 服务；
- 容器和仿真发布节点的 `ROS_DOMAIN_ID` 必须一致，但包内不固定具体数值。
