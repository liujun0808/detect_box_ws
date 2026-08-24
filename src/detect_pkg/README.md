# detect_pkg

detect_pkg 是一个基于 ROS 2、RealSense D435 和 YOLO-World 的开口塑料箱检测与三维中心定位包。

当前方案采用服务请求触发：

~~~text
/detect 请求
  -> RealSense SDK 获取对齐 RGB-D
  -> YOLO-World 检测箱体
  -> bbox 内深度反投影
  -> SOR 离群点过滤和三维聚类
  -> 远侧内壁、真实上边沿特征提取
  -> 固定尺寸箱体中心优化
  -> camera 到 base_link 外参转换
  -> 返回原服务响应
~~~

当前不使用：

- AprilTag；
- 连续帧运动差分；
- 传送带支撑平面；
- 箱体姿态估计；
- 传送带运动补偿。

详细算法、公式和流程图见：

~~~text
/home/ub/project/detect_box_ws/pointcloud_filter_and_position_optimization.md
~~~

## 1. 保留接口

服务名称和服务类型保持原有接口：

~~~text
service: /detect
type: upper_limb_interface/srv/DetectAprilTag
request: capture_once
response: success, message, box_pose
~~~

调用成功时：

- box_pose.position 是箱体中心在 base_link 中的位置；
- box_pose.orientation 当前使用单位四元数；
- message 包含类别、YOLO 置信度、camera/base_link 中心、点云数量和质量指标。

当前保留 box_pose 话题配置，默认话题名为：

~~~text
box_pose
~~~

## 2. 目录结构

~~~text
detect_box_ws/
├── src/
│   └── detect_pkg/
│       ├── CMakeLists.txt
│       ├── package.xml
│       ├── config/
│       │   └── box_position_estimation.yaml
│       ├── launch/
│       │   └── box_position_estimation.launch.py
│       ├── detect_box_pipeline/
│       │   ├── __init__.py
│       │   ├── detect_server_node.py
│       │   ├── realsense_camera.py
│       │   ├── yolo_world_detector.py
│       │   ├── box_position_estimator.py
│       │   └── debug_snapshot.py
│       ├── scripts/
│       │   ├── detect_server_node
│       │   └── print_box_pose_in_pelvis.py
│       └── src/
│           └── detect_client_node.cpp
├── models/
│   └── yolov8s-worldv2.pt
├── debug_box_position/
└── pointcloud_filter_and_position_optimization.md
~~~

Python 主实现位于 detect_box_pipeline 目录。scripts/detect_server_node 是 ROS 2 可执行入口包装器，用于确保节点使用配置的 Python 环境启动。launch 文件负责加载 YAML 参数并启动节点。

## 3. 主要依赖

### 3.1 系统依赖

- Ubuntu；
- ROS 2；
- ROS 2 package upper_limb_interface；
- librealsense2 和 RealSense D435；
- colcon、ament_cmake_python。

### 3.2 Python 依赖

当前 Python 主程序建议在 py310 conda 环境中运行，主要依赖：

~~~text
Python 3.10
numpy
scipy
opencv-python
pyrealsense2
ultralytics
torch
~~~

YOLO 使用 CUDA 时，还需要安装与显卡驱动匹配的 CUDA 版 PyTorch。

检查 Python 环境：

~~~bash
conda activate py310
python --version
python -c "import torch; print(torch.cuda.is_available())"
python -c "import pyrealsense2; print('pyrealsense2 ok')"
python -c "import ultralytics; print('ultralytics ok')"
~~~

如果缺少基础 Python 包：

~~~bash
conda activate py310
pip install numpy scipy opencv-python pyrealsense2 ultralytics
~~~

PyTorch 建议按照当前显卡驱动和 CUDA 版本选择对应安装命令，不要直接使用不匹配的 CPU 版本。

## 4. YOLO 权重

默认模型路径：

~~~text
/home/ub/project/detect_box_ws/models/yolov8s-worldv2.pt
~~~

检查：

~~~bash
ls -lh /home/ub/project/detect_box_ws/models/yolov8s-worldv2.pt
~~~

如果模型路径不同，可以修改：

~~~yaml
yolo_model_path: /absolute/path/to/yolov8s-worldv2.pt
~~~

当前提示词为：

~~~yaml
yolo_class_prompts:
  - green plastic crate
  - plastic crate
  - storage crate
  - green storage box
  - green box
  - plastic container
~~~

## 5. 参数文件

默认参数文件：

~~~text
src/detect_pkg/config/box_position_estimation.yaml
~~~

参数主要包括：

- RealSense 分辨率、帧率、预热和丢帧；
- YOLO 权重、设备、输入尺寸和置信度；
- 箱体尺寸和 box_coordinate_z_sign；
- 深度范围和 bbox 扩张；
- SOR 与三维聚类；
- 远侧内壁和真实上边沿 RANSAC；
- 固定尺寸中心优化；
- 原 camera 到 base_link 外参；
- 调试快照保存。

当前箱体尺寸：

~~~yaml
box_size_x_m: 0.295
box_size_y_m: 0.395
box_size_z_m: 0.225
~~~

当前外参参数 camera_to_base_row_major 保持原工程语义，不要改成逆矩阵。

## 6. 编译

首先 source ROS 2：

~~~bash
source /opt/ros/humble/setup.bash
~~~

如果 upper_limb_interface 已经安装或已经 source，只构建本包：

~~~bash
cd /home/ub/project/detect_box_ws
colcon build --packages-select detect_pkg --symlink-install
source install/setup.bash
~~~

如果接口包也需要在当前工作空间构建：

~~~bash
colcon build --packages-select upper_limb_interface detect_pkg --symlink-install
source install/setup.bash
~~~

如果 upper_limb_interface 的旧 build 目录导致符号链接冲突，需要清理该包对应的 build/install 产物后再构建，避免影响 detect_pkg 之外的用户修改。

## 7. 启动

### 7.1 前台启动

在一个新终端中执行以下完整命令：

~~~bash
cd /home/ub/project/detect_box_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch detect_pkg box_position_estimation.launch.py
~~~

使用指定参数文件：

~~~bash
ros2 launch detect_pkg box_position_estimation.launch.py \
  params_file:=/absolute/path/to/box_position_estimation.yaml
~~~

如果直接运行可执行入口，必须显式加载 YAML 参数：

~~~bash
ros2 run detect_pkg detect_server_node --ros-args \
  --params-file src/detect_pkg/config/box_position_estimation.yaml
~~~

推荐使用 launch，因为 launch 会自动加载默认 YAML 参数。

启动成功后保持该终端运行，再从另一个终端调用 /detect 服务。

### 7.2 使用 systemd 开机自启

如果需要开机自动启动检测节点，执行：

~~~bash
cd /home/ub/project/detect_box_ws
chmod +x scripts/start_ros_nodes.sh scripts/install_autostart.sh
./scripts/install_autostart.sh
~~~

自启服务使用与前台启动相同的 launch 文件，不会额外启动相机节点。完整的安装、查看日志和停止方法见：

~~~text
/home/ub/project/detect_box_ws/scripts/autostart_readme.md
~~~

## 8. 触发检测

服务请求：

~~~bash
ros2 service call /detect upper_limb_interface/srv/DetectAprilTag \
  "{capture_once: true}"
~~~

当已有请求正在执行时，新的请求不会并行进入相机和优化器。

检查服务：

~~~bash
ros2 service list | grep detect
ros2 service type /detect
~~~

## 9. 运行日志

启动时会打印 YOLO 设备选择结果。第一次成功推理后会打印实际推理设备，例如：

~~~text
YOLO device selected: cuda:0
YOLO actual inference device: cuda:0
~~~

每次检测还会打印阶段耗时：

~~~text
capture
yolo
pointcloud_extract
pointcloud_process
optimizer
pipeline
debug_snapshot
total
~~~

第一次请求可能较慢，因为包含相机启动、预热、硬件复位或 YOLO CUDA 初始化。相机保持运行后，后续请求通常明显更快。

## 10. 调试输出

默认开启调试保存：

~~~yaml
debug_enabled: true
debug_output_dir: /home/ub/project/detect_box_ws/debug_box_position
debug_max_snapshots: 10
~~~

每次请求创建一个时间命名目录，最多保留最近 10 次。当前保存：

~~~text
color_with_yolo_bbox.png
final_candidate_cloud.ply
~~~

color_with_yolo_bbox.png 包含 YOLO bbox、类别名称和置信度。

final_candidate_cloud.ply 使用 camera 坐标系，包含：

- 最终候选箱体真实点云；
- 一个红色顶点，表示最终估计的箱体中心。

当前不保存：

~~~text
depth_u16.png
completed_box_model_cloud.ply
yolo_roi_cloud.ply
~~~

关闭调试保存：

~~~yaml
debug_enabled: false
~~~

## 11. 常见问题

### YOLO 权重不存在

检查 yolo_model_path 和模型文件：

~~~bash
ls -lh /home/ub/project/detect_box_ws/models/yolov8s-worldv2.pt
~~~

### YOLO 没有使用 CUDA

查看节点日志中的：

~~~text
YOLO device selected
YOLO actual inference device
~~~

如果显示 cpu，需要检查 py310 环境中的 PyTorch 是否为 CUDA 版本，以及显卡驱动是否正常：

~~~bash
conda activate py310
python -c "import torch; print(torch.__version__); print(torch.cuda.is_available())"
nvidia-smi
~~~

### RealSense 没有帧

检查 D435 USB 连接、设备枚举和 librealsense 安装。当前首帧超时时会按照 YAML 配置尝试一次硬件复位：

~~~yaml
camera_reset_on_start_failure: true
~~~

### CloudCompare 看不到中心点

确认加载的是最新快照中的 final_candidate_cloud.ply，并在 CloudCompare 中：

1. 开启 RGB 颜色显示；
2. 增大点显示尺寸；
3. 查找红色点；
4. 确认坐标系是 camera，而不是 base_link。

### 结果质量不满足要求

优先检查：

- color_with_yolo_bbox.png 中 bbox 是否正确；
- 候选点云是否包含箱体主体；
- surface_rmse_m；
- surface_inlier_ratio；
- inner_wall 是否为 valid；
- 上边沿是否被可靠捕捉。

不要先随意增大优化权重，应先确认输入点云确实来自箱体。

## 12. 接口和外参注意事项

以下内容不能随意修改：

~~~text
服务名：/detect
服务类型：upper_limb_interface/srv/DetectAprilTag
输出坐标系：base_link
camera_to_base_row_major 的矩阵方向
箱体尺寸
~~~

当前输出的是箱体中心位置，不是箱体 6D 姿态。方向字段使用单位四元数只是为了保持原 Pose 接口结构。
