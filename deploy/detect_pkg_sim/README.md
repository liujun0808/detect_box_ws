# detect_pkg_sim 仿真检测部署包

本目录用于在 x86_64 NVIDIA Linux 主机上运行已发布的 `detect_pkg_sim` Docker 镜像。部署包不包含检测源码、模型权重、Conda、PyTorch 或 ROS 2 编译产物；它们全部封装在镜像内。

镜像启动一个 `/detect` ROS 2 服务节点。节点订阅仿真 D435 RGB-D 话题，收到服务请求后等待一组新的同步 RGB-D 帧，执行 YOLO-World 检测、点云处理和固定尺寸箱体中心估计，然后通过原服务接口返回结果。

## 目录结构

```text
detect_pkg_sim/
├── README.md                              # 本说明
├── compose.yaml                           # 容器网络、GPU、外部参数和调试目录挂载
├── .env.example                           # 镜像版本、ROS Domain 等运行配置模板
├── .env                                  # 本机实际配置，由用户从模板复制，不纳入版本管理
├── config/
│   └── box_position_estimation_sim.yaml  # 可编辑的运行参数
├── debug_box_position_sim/               # 自动生成的最近十次调试快照
└── scripts/
    ├── compose.sh                         # Compose 公共包装器，自动处理文件权限
    ├── pull_image.sh                      # 拉取 .env 中指定的镜像版本
    ├── start.sh                           # 启动或重建检测容器
    ├── stop.sh                            # 停止并移除检测容器
    ├── logs.sh                            # 持续查看检测节点日志
    ├── status.sh                          # 查看容器状态
    └── call_detect_from_container.sh      # 容器内服务自检
```

同事只需要保留整个 `detect_pkg_sim/` 目录。`config/` 和 `.env` 是本机可修改部分；更新镜像时不要覆盖已调好的本机 YAML。

## 运行原理

Compose 以宿主网络、GPU 和宿主 IPC 启动容器：

```text
仿真相机 ROS 2 节点
    ├── /camera/color/image_raw                  sensor_msgs/Image, rgb8
    ├── /camera/color/camera_info                 sensor_msgs/CameraInfo
    └── /camera/aligned_depth_to_color/image_raw  sensor_msgs/Image, 32FC1, m
                         │
                         ▼
Docker: detect_pkg_sim /detect 服务
                         │
                         ▼
upper_limb_interface/srv/DetectAprilTag 响应
```

容器使用 ROS 2 Humble 默认的 Fast DDS，但容器内设置 `FASTDDS_BUILTIN_TRANSPORTS=UDPv4`。这会避免 Fast DDS 在同一物理机上优先选择跨 Docker 边界不可靠的共享内存传输；宿主机的仿真、MoveIt 和其他 Fast DDS 节点无需切换 DDS 实现或额外设置 DDS 环境变量。

## 系统要求

- x86_64 Ubuntu 主机；
- 可用的 NVIDIA 显卡与驱动，`nvidia-smi` 必须正常；
- Docker Engine、Docker Compose v2 插件；
- NVIDIA Container Toolkit；
- 与相机发布节点相同的 ROS 2 Domain，默认 `0`；
- 仿真发布端持续发布上面列出的三个话题；
- 若需要从宿主机其他 ROS 2 项目调用 `/detect`，调用端必须具有完全相同的 `upper_limb_interface/srv/DetectAprilTag` 服务定义。

宿主机不需要安装 Conda、Python PyTorch、YOLO、CLIP、OpenCV、RealSense SDK 或模型权重。只有宿主机 ROS 2 客户端需要 ROS 2 Humble 和 `upper_limb_interface` 接口包。

## 首次安装

### 1. 验证 NVIDIA 驱动

```bash
nvidia-smi
```

必须显示 GPU、驱动版本和显存。容器自带 CUDA PyTorch 用户态库，但不能替代宿主机 NVIDIA 驱动。

### 2. 安装 Docker Engine 与 Compose

以下命令适用于 Ubuntu，使用 Docker 官方 apt 软件源：

```bash
sudo apt-get update
sudo apt-get install -y ca-certificates curl
sudo install -m 0755 -d /etc/apt/keyrings
sudo curl -fsSL https://download.docker.com/linux/ubuntu/gpg \
  -o /etc/apt/keyrings/docker.asc
sudo chmod a+r /etc/apt/keyrings/docker.asc

source /etc/os-release
echo \
  "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.asc] https://download.docker.com/linux/ubuntu ${UBUNTU_CODENAME:-$VERSION_CODENAME} stable" \
  | sudo tee /etc/apt/sources.list.d/docker.list >/dev/null

sudo apt-get update
sudo apt-get install -y \
  docker-ce docker-ce-cli containerd.io \
  docker-buildx-plugin docker-compose-plugin
sudo systemctl enable --now docker
sudo usermod -aG docker "$USER"
```

注销 Ubuntu 图形会话并重新登录后验证：

```bash
docker version
docker compose version
docker run --rm hello-world
```

### 3. 安装 NVIDIA Container Toolkit

```bash
sudo apt-get update
sudo apt-get install -y --no-install-recommends ca-certificates curl gnupg2

curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey \
  | sudo gpg --dearmor \
    -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg

curl -s -L \
  https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list \
  | sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' \
  | sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list

sudo apt-get update
sudo apt-get install -y nvidia-container-toolkit
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker
```

验证 GPU 能传入容器：

```bash
docker run --rm --gpus all \
  nvidia/cuda:12.4.1-base-ubuntu22.04 nvidia-smi
```

输出必须包含宿主机 GPU。Docker 与 NVIDIA Container Toolkit 的详细官方安装说明分别见 [Docker Engine for Ubuntu](https://docs.docker.com/engine/install/ubuntu/) 和 [NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html)。

## 拉取与启动

进入部署目录，首次复制本机环境文件：

```bash
cd /path/to/detect_pkg_sim
cp .env.example .env
```

默认镜像为：

```text
liujun0808/detect_pkg_sim:v0.1.0
```

仓库为公有，拉取并启动：

```bash
./scripts/pull_image.sh
./scripts/start.sh
./scripts/logs.sh
```

当日志出现以下内容时，服务已就绪：

```text
Simulation box-position service ready: service=/detect
```

`logs.sh` 只显示日志；按 `Ctrl-C` 只退出日志查看，不会停止容器。

常用管理命令：

```bash
./scripts/status.sh
./scripts/stop.sh
./scripts/start.sh
```

`start.sh` 会重建容器但不会重新下载或构建镜像，因此修改 YAML 后执行一次 `start.sh` 即可加载新参数。

## 服务调用

### 部署自检

无需在宿主机安装 ROS 2 接口包，先用容器内客户端检查服务：

```bash
./scripts/call_detect_from_container.sh
```

### 由宿主机 ROS 2 项目调用

调用端需 source ROS 2 与包含 `upper_limb_interface` 的工作空间：

```bash
source /opt/ros/humble/setup.bash
source /path/to/interface_workspace/install/setup.bash

ros2 service call /detect upper_limb_interface/srv/DetectAprilTag \
  "{capture_once: true}"
```

调用端不需要设置 `RMW_IMPLEMENTATION`；默认 Fast DDS 即可。调用端、相机发布端与容器只需使用相同的 `ROS_DOMAIN_ID`，默认均为 `0`。

成功响应中的 `box_pose.position` 是箱体中心在 `base_link` 下的位置。四元数保留原接口兼容性；当前方案只估计中心位置，不估计箱体姿态。

## 参数调整

所有可调参数位于：

```text
config/box_position_estimation_sim.yaml
```

该 YAML 以只读方式挂载到容器，并由 launch 的 `params_file` 参数加载。常见调整项：

- `camera_to_base_row_major`：仿真相机到 `base_link` 的 4x4 外参；必须与仿真场景一致；
- `yolo_confidence_threshold`：YOLO 置信度阈值；
- `depth_min_m`、`depth_max_m`：有效深度范围；
- `depth_bbox_margin_px`：YOLO 框的深度 ROI 外扩像素；
- `debug_enabled`、`debug_max_snapshots`：调试保存开关和滚动保留数量。

修改后重启服务：

```bash
./scripts/start.sh
```

YAML 模板应与镜像版本对应。算法、服务定义、Python 依赖、模型权重或镜像内默认参数变动时，需要发布新镜像版本；仅修改本机 YAML 不需要重新构建或重新拉取镜像。

## 调试文件

每个 `/detect` 请求默认在以下目录创建时间戳子目录：

```text
debug_box_position_sim/<时间戳>/
```

成功估计时通常包括：

```text
color_raw.png
color_with_yolo_bbox.png
final_candidate_cloud.ply
```

`final_candidate_cloud.ply` 是带颜色的最终候选点云，其中红色点为最终估计的箱体中心。系统滚动保留最近十次快照。启动脚本会让这些文件归宿主机执行用户所有，可直接查看和删除。

## 镜像升级

编辑 `.env` 中的明确版本标签，例如：

```text
DETECT_BOX_SIM_IMAGE=liujun0808/detect_pkg_sim:v0.1.1
```

然后执行：

```bash
./scripts/pull_image.sh
./scripts/start.sh
```

Docker 只下载本机没有的镜像层。请使用明确的 `vX.Y.Z` 标签，不建议长期固定使用 `latest`。

## 故障排查

| 现象 | 排查方式 |
|---|---|
| `permission denied ... docker.sock` | 执行 `sudo usermod -aG docker "$USER"` 后完整注销并重新登录。 |
| 容器无法启动或 GPU 不可见 | 依次检查 `nvidia-smi` 与 `docker run --rm --gpus all nvidia/cuda:12.4.1-base-ubuntu22.04 nvidia-smi`。 |
| `/detect` 找不到 | 检查 `.env` 的 `ROS_DOMAIN_ID` 与相机发布端一致，并执行 `./scripts/logs.sh`。 |
| 服务返回 RGB-D 超时 | 确认三个相机话题名称、编码、时间戳与 YAML 完全一致。 |
| 服务返回 `YOLO found no crate` | 查看 `color_with_yolo_bbox.png`，再调整提示词或 `yolo_confidence_threshold`。 |
| `base_link` 位置不正确 | 核对 `camera_to_base_row_major` 是否为仿真场景的真实外参。 |
| 无法删除调试目录 | 通过本部署包的 `scripts/start.sh` 启动，避免直接以 root 用户执行 Compose。 |

