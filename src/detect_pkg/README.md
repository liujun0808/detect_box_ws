# detect_pkg

`detect_pkg` 是一个 ROS2 C++ 检测服务包，用于从 RealSense D435 彩色图像中识别 AprilTag，并根据 box 上预设的两个 tag 估计 box 在 `base_link` 坐标系下的位姿。目前使用的外参矩阵 `camera2base` 来自 MuJoCo 模型，部署到实机需要重新标定。

## 在视觉抓取流程中的作用

视觉抓取流程中，本包只负责一件事：收到服务请求后，从最近一帧 D435 彩色图像中检测箱子 AprilTag，并返回箱子在 `base_link` 坐标系下的 `geometry_msgs/Pose`。后续抓取点计算、双臂控制、吸附和放置都由 `qi_robot_moveit_py/vision_grasp_sequence` 完成。

仿真环境推荐启动顺序：

```bash
cd ~/qi_deploy
source install/setup.bash
ros2 launch mujoco_d435_publisher d435_camera_publisher.launch.py
```

另开终端：

```bash
cd ~/qi_deploy
source install/setup.bash
ros2 run detect_pkg detect_server_node
```

另开终端检查检测结果：

```bash
cd ~/qi_deploy
source install/setup.bash
ros2 run detect_pkg detect_client_node
```

确认检测成功后，再运行抓取程序：

```bash
ros2 run qi_robot_moveit_py vision_grasp_sequence
```

返回坐标系说明：

```text
AprilTag 图像检测
  -> box2camera
  -> camera2base 外参
  -> box_pose in base_link
```

当前 `camera2base` 外参写在 `detect_sever_node.cpp` 中，来自 MuJoCo 仿真相机。换真实相机或调整相机安装位置时，必须重新标定并更新该外参。


当前实现采用相机驱动与检测服务分离的架构：

- `realsense2_camera` 官方 ROS2 驱动负责启动 D435、发布 RGB 图像和相机内参。
- 使用mujoco环境中的相机时，使用该包`mujoco_d435_publisher`去启动相机节点
- `detect_server_node` 只订阅图像和内参，不直接链接 `librealsense2`。
- 服务请求到来时，检测节点使用最近缓存的一帧图像进行 AprilTag 识别和 box 位姿估计。

这样可以避免 `librealsense2` 与 ROS2 默认 FastDDS 在同一进程内的符号冲突问题。

## 目录结构

```text
detect_pkg/
├── CMakeLists.txt
├── package.xml
├── README.md
├── include/detect_pkg/detect.h
├── launch/detect_with_realsense.launch.py
└── src/
    ├── detect_sever_node.cpp
    └── detect_client_node.cpp
```

## 依赖

ROS2 包依赖：

- `rclcpp`
- `sensor_msgs`
- `geometry_msgs`
- `upper_limb_interface`
- `realsense2_camera`
- `launch`
- `launch_ros`

系统/第三方库依赖：

- OpenCV，包含 `aruco`、`calib3d`、`highgui`、`imgcodecs`
- Eigen3

安装 RealSense ROS2 驱动：

```bash
sudo apt install ros-humble-realsense2-camera
```

## 编译

在工作空间根目录执行：

```bash
colcon build --packages-select upper_limb_interface detect_pkg
source install/setup.bash
```

## 启动

在mujoco环境中，仿真环境启动后，先启动仿真相机：

```bash
ros2 launch mujoco_d435_publisher d435_camera_publisher.launch.py
```
再另开一个终端启动检测服务：

```bash
ros2 run detect_pkg detect_server_node
```

(仅限有实际相机才可使用)，使用 launch 同时启动相机驱动和检测服务：

```bash
ros2 launch detect_pkg detect_with_realsense.launch.py
```
## RealSense 启动配置

`detect_with_realsense.launch.py` 默认启动官方 `realsense2_camera` 驱动，并使用如下配置：

```text
enable_color: true
enable_depth: false
enable_infra1: false
enable_infra2: false
rgb_camera.color_profile: 640x480x15
```

默认关闭深度图和红外图，只保留彩色图像。彩色图像默认使用 `640x480x15`，原因是 D435 在 USB 供电不稳定或总线带宽不足时，高分辨率、高帧率、多数据流同时传输容易导致相机传输失败。现场使用时建议使用外部供电的 USB 3.0 扩展坞，避免相机掉帧或取流失败。

如需调整彩色图像规格：

```bash
ros2 launch detect_pkg detect_with_realsense.launch.py color_profile:=640x480x30
```

## Topic 约定

检测服务端默认订阅：

```text
/camera/camera/color/image_raw
/camera/camera/color/camera_info
```

有实际相机时，可通过 launch 参数覆盖：
```bash
ros2 launch detect_pkg detect_with_realsense.launch.py \
  image_topic:=/camera/camera/color/image_raw \
  camera_info_topic:=/camera/camera/color/camera_info
```

服务端目前支持的图像编码：

- `bgr8`
- `rgb8`

## 服务接口

服务定义位于：

```text
upper_limb_interface/srv/DetectAprilTag.srv
```

接口内容：

```srv
bool capture_once
---
bool success
string message
geometry_msgs/Pose box_pose
```

字段说明：

- `capture_once`：触发一次检测。当前服务端不区分该字段真假，收到请求即使用最近缓存图像进行检测。
- `success`：是否成功估计到 box 位姿。
- `message`：检测结果说明或错误原因。
- `box_pose`：box 坐标系在 `base_link` 坐标系下的位姿。服务端内部先估计 `box2camera`，再使用 `camera2base` 外参转换到 `base_link`。失败时返回单位位姿：

```text
position = [0, 0, 0]
orientation = [0, 0, 0, 1]
```

服务名默认：

```text
/detect
```

可通过参数修改：

```bash
ros2 launch detect_pkg detect_with_realsense.launch.py service_name:=detect
```

## 测试客户端

包内提供了一个手动测试客户端：

```bash
ros2 run detect_pkg detect_client_node
```

客户端启动 3 秒后，会在终端询问是否发送请求：

```text
是否向检测服务发送一次请求？[y/n]:
```

输入 `y` 后发送一次服务请求，收到响应后打印：

- `success`
- `message`
- `box_pose.position`
- `box_pose.orientation`

然后继续询问是否发送下一次请求。输入 `n` 退出。

## 仿真 box 相对 pelvis 位姿

在启动 MuJoCo/MoveIt 仿真后，另开一个已 `source install/setup.bash` 的终端执行：

```bash
ros2 run detect_pkg print_box_pose_in_pelvis.py
```

脚本实时查询 TF 中的 `T_pelvis_box`，打印 box 原点在 `pelvis` 坐标系下的位置和四元数姿态（`x, y, z, w`）。默认每秒打印 10 次；可通过参数调整：

```bash
ros2 run detect_pkg print_box_pose_in_pelvis.py --ros-args -p print_rate_hz:=20.0
```

该功能依赖 MuJoCo 仿真器发布 `world -> pelvis` 和 `world -> table_object_box` 动态 TF。

## 检测参数

服务端主要参数：

```text
service_name: detect
image_topic: /camera/camera/color/image_raw
camera_info_topic: /camera/camera/color/camera_info
tag_size_m: 0.0625
box_tag_ids: [10, 24]
box_tag_positions_m: [-0.041, 0.075, 0.0, -0.041, -0.003, 0.0]
box_tag_rotations_row_major: [1, 0, 0, 0, 1, 0, 0, 0, 1, 1, 0, 0, 0, 1, 0, 0, 0, 1]
enable_debug_image: false
debug_window_name: AprilTag Detection
debug_image_save_prefix: apriltag_detection
```

其中 `service_name`、`image_topic`、`camera_info_topic`、`enable_debug_image` 由 `detect_with_realsense.launch.py` 暴露为启动参数。

`tag_size_m`、`box_tag_ids`、`box_tag_positions_m`、`box_tag_rotations_row_major`、`debug_window_name`、`debug_image_save_prefix` 属于检测算法和标定相关的内部参数，当前不通过 launch 文件配置。如需修改，请在 `detect_sever_node.cpp` 的构造函数默认值中调整，并重新编译。

`tag_size_m` 表示 OpenCV 实际识别的黑色 AprilTag 方框边长，单位为米，必须与图案黑色外边框一致。当前 MuJoCo 资产的 mesh 边长是 0.08 m，但其 PNG 纹理有白边，黑色方框占 400/512，因此默认值为 `0.08 * 400 / 512 = 0.0625 m`。

`box_tag_ids` 表示用于定位 box 的 tag ID。

`box_tag_positions_m` 表示每个 tag 原点在 box 坐标系下的位置，按 `[x1, y1, z1, x2, y2, z2, ...]` 排列，单位为米。

`box_tag_rotations_row_major` 表示每个 tag 坐标系相对 box 坐标系的旋转矩阵 `R_box_tag`，按每个矩阵 9 个元素的行优先顺序排列：`[r00, r01, r02, r10, r11, r12, r20, r21, r22, ...]`。矩阵将 tag 坐标系中的向量转换到 box 坐标系，必须满足 `R^T R = I` 和 `det(R) = 1`。默认值为两个单位矩阵，分别对应 tag 10 和 tag 24。


## 调试图像

开启：

```bash
ros2 launch detect_pkg detect_with_realsense.launch.py enable_debug_image:=true
```

开启后服务端会：

- 显示 OpenCV 图像窗口。
- 绘制 AprilTag 边框和 ID。
- 绘制 tag 坐标轴。
- 在图像左上角绘制检测状态文字。
- 服务返回后保存一张调试图像到节点运行目录。

保存文件名示例：

```text
apriltag_detection_success_1781663537802474571.png
apriltag_detection_failed_1781663537802474571.png
```

如需修改调试窗口名或保存文件名前缀，请在源码中调整 `debug_window_name` 或 `debug_image_save_prefix` 默认值，并重新编译。

注意：调试窗口需要图形桌面环境。如果通过 SSH 或无显示环境运行，建议关闭 `enable_debug_image`。

## 常见问题

### 1. 服务返回“尚未收到相机彩色图像”

确认 RealSense 驱动是否正常发布图像：

```bash
ros2 topic list | grep color
ros2 topic hz /camera/camera/color/image_raw
```

如果实际话题名不同，请通过 `image_topic` 和 `camera_info_topic` 参数覆盖。

### 2. 服务返回“尚未收到相机内参”

确认相机内参话题：

```bash
ros2 topic echo /camera/camera/color/camera_info --once
```

### 3. 识别不到 AprilTag

检查：

- tag 字典是否为 `DICT_APRILTAG_36h11`
- `tag_size_m` 是否与真实 tag 边长一致
- `box_tag_ids` 是否与实际 tag ID 一致
- 图像是否清晰、曝光是否合适
- tag 是否完整出现在画面中

### 4. 相机传输失败或掉帧

建议：

- 使用外部供电的 USB 3.0 扩展坞
- 保持默认 `640x480x15`
- 不开启深度图和红外图
- 避免和其他高带宽 USB 设备共用同一路控制器

### 5. FastDDS 与 RealSense 冲突

当前服务端已经不直接链接 `librealsense2`，正常情况下不会再触发该冲突。RealSense 由 `realsense2_camera` 独立进程负责，检测服务端只通过 ROS topic 获取图像和内参。
