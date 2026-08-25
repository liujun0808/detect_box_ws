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

本节分为两部分：

- 3.1、3.2 是普通 Ubuntu/PC 平台的通用依赖说明，也适用于已经具备对应软件环境的 Jetson；
- Jetson Orin NX 不要直接照搬 3.2 中的通用 `torch` 和 `pyrealsense2` 安装命令，应按照 3.3 的 Jetson 专用流程安装；
- 你当前的 Orin NX 已经安装 ROS 2 Humble，因此不需要重复安装 ROS 2，只需要执行 3.3.1 中的环境确认和系统依赖安装。

### 3.1 系统依赖

- Ubuntu；
- ROS 2；
- ROS 2 package upper_limb_interface；
- librealsense2 和 RealSense D435；
- colcon、ament_cmake_python。

### 3.2 普通 Ubuntu/PC 平台的 Python 依赖

这一小节主要面向普通 Ubuntu/PC 平台，或者已经确认存在对应 Python wheel 的平台。当前 Python 主程序建议在 py310 conda 环境中运行，主要依赖：

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

普通 Ubuntu/PC 平台可以根据显卡和 CUDA 版本安装 PyTorch。Jetson Orin NX 不要执行这里的通用 `pip install torch` 或直接使用 PC 的 PyTorch wheel，必须执行 3.3.3 中与 JetPack 匹配的 NVIDIA Jetson PyTorch 安装流程。

### 3.3 Jetson Orin NX 平台部署

以下流程针对已经安装 JetPack 和 ROS 2 Humble 的 Jetson Orin NX。Jetson NX 不能直接套用普通 x86 Ubuntu 主机的 PyTorch 安装命令，请先确认硬件、JetPack 和 ROS 2 环境：

~~~bash
uname -m
cat /etc/nv_tegra_release
cat /etc/os-release
python3 --version
ls /opt/ros
~~~

正常应看到：

- 架构为 `aarch64`；
- 硬件为 `Jetson Orin NX`；
- `/opt/ros` 中包含 `humble`。

当前目标环境：

| 硬件 | 推荐系统 | ROS 2 建议 | 说明 |
|---|---|---|---|
| Orin NX | JetPack 6.x / Ubuntu 22.04 | ROS 2 Humble | 与 Humble 的 Ubuntu 22.04 arm64 支持最匹配 |

JetPack 会同时影响 Jetson Linux、CUDA、TensorRT 和 PyTorch 的可用版本。安装或升级 JetPack 后，应重新确认 Python 深度学习环境，不要沿用另一台机器的 `torch` 安装包。可参考 NVIDIA 的 [JetPack 6.1](https://developer.nvidia.com/embedded/jetpack-sdk-61)、[Jetson PyTorch 安装说明](https://docs.nvidia.com/deeplearning/frameworks/install-pytorch-jetson-platform/index.html) 和 [ROS 2 Humble 平台支持列表](https://docs.ros.org/en/humble/Releases/Release-Humble-Hawksbill.html)。

#### 3.3.1 安装 Jetson 系统依赖

你已经安装 ROS 2 Humble，因此不需要再次执行 `sudo apt install ros-humble-desktop`。先确认 ROS 2 环境：

~~~bash
source /opt/ros/humble/setup.bash
ros2 --version
ls /opt/ros
~~~

然后安装本包所需、但不属于 ROS 2 的系统依赖：

~~~bash
sudo apt update
sudo apt install -y \
  build-essential cmake git pkg-config \
  python3-dev python3-pip python3-venv \
  python3-colcon-common-extensions python3-rosdep python3-vcstool \
  libopenblas-dev libusb-1.0-0-dev libudev-dev libssl-dev udev
~~~

确认 `ros2 --version` 可执行，且 `/opt/ros` 中存在 `humble`。`upper_limb_interface` 也必须已经安装，或位于当前工作空间的 `src` 目录中。

如果系统尚未初始化 rosdep，再执行以下命令；已经初始化过则跳过 `rosdep init`：

~~~bash
sudo rosdep init
rosdep update
~~~

#### 3.3.2 创建 NX 上的 py310 环境

建议在 NX 上使用支持 `aarch64` 的 Miniforge/Miniconda，然后创建与当前代码一致的 Python 3.10 环境：

~~~bash
conda create -n py310 python=3.10 -y
conda activate py310
python -m pip install --upgrade pip
~~~

确认 Python 架构和版本：

~~~bash
python -c "import platform, sys; print(platform.machine()); print(sys.executable); print(sys.version)"
~~~

输出的架构应为 `aarch64`，解释器应位于 NX 上实际存在的 `py310` 环境中。

#### 3.3.3 安装 Jetson CUDA 版 PyTorch

Jetson 上必须安装 NVIDIA 针对 JetPack 发布的 `aarch64` PyTorch wheel，并且 wheel 版本要和当前 JetPack 匹配。不要执行没有版本约束的：

~~~bash
pip install torch
~~~

该命令可能安装不适用于 Jetson 的通用包，或者覆盖已经正确安装的 NVIDIA 版本。请按照 NVIDIA 的 [Jetson PyTorch 安装说明](https://docs.nvidia.com/deeplearning/frameworks/install-pytorch-jetson-platform/index.html)选择与 JetPack 对应的安装包，然后在 `py310` 环境中执行 NVIDIA 给出的安装命令。例如，官方文档通常要求先设置对应的 wheel 地址，再安装：

~~~bash
conda activate py310
# 按 NVIDIA 文档中与当前 JetPack 匹配的版本填写 TORCH_INSTALL
export TORCH_INSTALL=/absolute/path/or/url/to/jetson_pytorch_wheel.whl
python -m pip install --no-cache-dir "${TORCH_INSTALL}"
~~~

安装后验证 CUDA：

~~~bash
python - <<'PY'
import torch
print("torch:", torch.__version__)
print("cuda available:", torch.cuda.is_available())
if torch.cuda.is_available():
    print("device:", torch.cuda.get_device_name(0))
PY
~~~

如果 `cuda available` 为 `False`，先不要启动 ROS 节点，优先检查 JetPack、wheel、驱动和环境是否匹配。

#### 3.3.4 安装 Ultralytics 和 RealSense Python 接口

在已经安装好 Jetson PyTorch 的同一个环境中安装其余 Python 包：

~~~bash
conda activate py310
python -m pip install numpy scipy opencv-python ultralytics
python -m pip install pyrealsense2
~~~

验证：

~~~bash
python -c "import cv2, numpy, scipy, torch, ultralytics; print('python packages ok')"
python -c "import pyrealsense2 as rs; print('pyrealsense2 ok:', rs.__file__)"
~~~

如果 `pyrealsense2` 没有适用于当前 Jetson Python/架构的 pip wheel，需要从 librealsense 源码编译 Python binding。官方 Python binding 和 Jetson 安装说明分别见 [RealSense Python wrapper](https://github.com/IntelRealSense/librealsense/blob/master/wrappers/python/readme.md) 和 [RealSense Jetson installation](https://github.com/IntelRealSense/librealsense/blob/master/doc/installation_jetson.md)。基本流程如下：

~~~bash
sudo apt update
sudo apt install -y libusb-1.0-0-dev libudev-dev pkg-config \
  libssl-dev libgtk-3-dev libglfw3-dev libglu1-mesa-dev

git clone https://github.com/realsenseai/librealsense.git
cd librealsense
./scripts/setup_udev_rules.sh
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release \
  -DBUILD_EXAMPLES=false \
  -DBUILD_GRAPHICAL_EXAMPLES=false \
  -DBUILD_PYTHON_BINDINGS=true \
  -DPYTHON_EXECUTABLE="$(which python)"
make -j"$(nproc)"
sudo make install
sudo ldconfig
export PYTHONPATH="/usr/local/lib:${PYTHONPATH:-}"
python -c "import pyrealsense2 as rs; print('pyrealsense2 ok:', rs.__file__)"
~~~

RealSense SDK 直接由本包通过 `pyrealsense2` 获取 D435 图像，不需要启动 `realsense2_camera` ROS 话题节点。D435 应连接到 NX 的 USB 3.x 接口；如果出现设备可枚举但没有帧，先检查 USB 线缆、供电、udev 规则和 librealsense 的 Jetson backend 配置。

#### 3.3.5 准备权重和工作空间

将 `yolov8s-worldv2.pt` 放入工作空间的 `models` 目录：

~~~bash
mkdir -p /home/ub/project/detect_box_ws/models
cp /absolute/path/to/yolov8s-worldv2.pt \
  /home/ub/project/detect_box_ws/models/yolov8s-worldv2.pt
ls -lh /home/ub/project/detect_box_ws/models/yolov8s-worldv2.pt
~~~

如果 NX 上的用户名或工作空间路径不是 `/home/ub/project/detect_box_ws`，需要同时修改 YAML 中的绝对路径、launch 使用的参数路径以及自启脚本中的工作空间路径。

#### 3.3.6 在 NX 上编译

~~~bash
cd /home/ub/project/detect_box_ws
source /opt/ros/humble/setup.bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select upper_limb_interface detect_pkg --symlink-install
source install/setup.bash
~~~

如果 `upper_limb_interface` 已经安装，不需要在当前工作空间重复构建：

~~~bash
colcon build --packages-select detect_pkg --symlink-install
source install/setup.bash
~~~

#### 3.3.7 使用 NX 上的正确 Python 启动

当前 `scripts/detect_server_node` 默认尝试使用：

~~~text
/home/ub/miniconda3/envs/py310/bin/python
~~~

NX 上如果 Miniforge/Miniconda 安装路径不同，启动前必须将 `DETECT_BOX_PYTHON` 指向实际解释器：

~~~bash
conda activate py310
export DETECT_BOX_PYTHON="$(which python)"
cd /home/ub/project/detect_box_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch detect_pkg box_position_estimation.launch.py
~~~

也可以直接写绝对路径：

~~~bash
export DETECT_BOX_PYTHON=/home/<nx-user>/miniforge3/envs/py310/bin/python
~~~

配置 systemd 自启时，不能依赖交互式 `conda activate`。请在 `scripts/start_ros_nodes.sh` 中、执行 `ros2 launch` 之前加入实际的 `DETECT_BOX_PYTHON` 设置，然后重新安装自启服务：

~~~bash
cd /home/ub/project/detect_box_ws
sed -n '1,120p' scripts/start_ros_nodes.sh
# 编辑脚本，加入：
# export DETECT_BOX_PYTHON=/home/<nx-user>/miniforge3/envs/py310/bin/python
./scripts/install_autostart.sh
sudo systemctl restart detect_box_ws.service
~~~

启动后应在第一次推理日志中看到：

~~~text
YOLO actual inference device: cuda:0
~~~

若显示 `cpu`，先回到 3.3.3 检查 PyTorch，不要通过修改 YAML 把 CPU 误认为 CUDA 已启用。

#### 3.3.8 NX 启动和检测

前台启动：

~~~bash
cd /home/ub/project/detect_box_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
export DETECT_BOX_PYTHON=/home/<nx-user>/miniforge3/envs/py310/bin/python
ros2 launch detect_pkg box_position_estimation.launch.py
~~~

另开终端调用服务：

~~~bash
cd /home/ub/project/detect_box_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 service call /detect upper_limb_interface/srv/DetectAprilTag \
  "{capture_once: true}"
~~~

首次请求可能包含相机启动、首帧等待和 CUDA/模型初始化，因此耗时明显较长；相机保持运行后，后续请求通常会明显变快。NX 如果需要观察温度和负载，可使用：

~~~bash
tegrastats
~~~

`nvpmodel` 和 `jetson_clocks` 会改变功耗、频率和温度，只有确认散热和供电满足要求后才按具体 NX 载板说明使用，不要盲目固定为某个模式。

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

节点第一次成功推理后会打印实际推理设备，例如：

~~~text
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

查看节点日志中的实际推理设备：

~~~text
YOLO actual inference device
~~~

如果显示 `cpu`，需要检查 py310 环境中的 PyTorch 是否为 CUDA 版本，以及显卡驱动是否正常：

~~~bash
conda activate py310
python -c "import torch; print(torch.__version__); print(torch.cuda.is_available())"
~~~

普通 NVIDIA 主机可使用 `nvidia-smi`；Jetson NX 应优先使用 `tegrastats`，因为 Jetson 通常不提供桌面 NVIDIA 驱动中的 `nvidia-smi` 命令。

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
