# detect_pkg_sim Docker 部署

本目录只构建仿真检测镜像。最终镜像包含 `detect_pkg_sim` 和 `upper_limb_interface`，不包含实体 `detect_pkg`，也不包含 RealSense SDK 或 USB 相机依赖。

## 文件

```text
docker/
├── Dockerfile.sim
├── Dockerfile.sim.dockerignore
├── compose.sim.yaml
├── compose.sim.build.yaml
├── entrypoint.sim.sh
├── requirements-sim.txt
└── .env.example
```

## 镜像内容

- Ubuntu 22.04 和 ROS 2 Humble；
- Python 3.10；
- CUDA 12.4 对应的 PyTorch 2.6.0 和 torchvision 0.21.0；
- Ultralytics 8.4.80、CLIP、NumPy、SciPy、OpenCV；
- `cv_bridge`、`message_filters`、ROS 2 消息依赖；
- `upper_limb_interface` 和 `detect_pkg_sim` 的编译安装结果；
- `yolov8s-worldv2.pt` 和 `ViT-B-32.pt`。

镜像构建时检查两个权重文件的 SHA256，运行时不下载模型。

## 同事机器需要安装的内容

直接拉取预构建镜像的机器只需要：

1. x86_64 Ubuntu；
2. 可正常工作的 NVIDIA 驱动；
3. Docker Engine；
4. Docker Compose v2 插件；
5. NVIDIA Container Toolkit；
6. 能与仿真 ROS 2 节点通信的网络环境。

宿主机不需要安装 Conda、PyTorch、Ultralytics、CLIP、OpenCV、NumPy 或 SciPy。宿主机只有在本机直接运行仿真或调用 ROS 2 服务时才需要 ROS 2 Humble。

### 1. 验证 NVIDIA 驱动

先在宿主机执行：

```bash
nvidia-smi
```

必须能显示 GPU、驱动版本和显存信息。驱动不可用时不要继续配置容器；Docker 镜像自带 PyTorch CUDA 运行库，但不能替代宿主机 NVIDIA 驱动。

### 2. 安装 Docker Engine 和 Compose v2

以下命令面向 Ubuntu，使用 Docker 官方 apt 仓库：

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
```

允许当前用户运行 Docker：

```bash
sudo usermod -aG docker "$USER"
```

执行后注销并重新登录，再验证：

```bash
docker version
docker compose version
docker run --rm hello-world
```

详细说明见 [Docker Engine Ubuntu 安装文档](https://docs.docker.com/engine/install/ubuntu/) 和 [Docker Compose 插件安装文档](https://docs.docker.com/compose/install/linux/)。

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

详细说明见 [NVIDIA Container Toolkit 官方安装文档](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html)。

### 4. 验证容器 GPU

安装完成后执行：

```bash
docker version
docker compose version
docker run --rm --gpus all \
  nvidia/cuda:12.4.1-base-ubuntu22.04 nvidia-smi
```

第三条命令必须能够看到 NVIDIA GPU。若失败，应先修复宿主驱动或 NVIDIA Container Toolkit，再启动检测镜像。

## 本地构建

当前尚未发布远程镜像时执行：

```bash
cd /home/ub/project/detect_box_ws
./scripts/build_sim_image.sh
```

默认生成：

```text
detect-box-sim:local
```

构建镜像需要访问 Ubuntu、ROS、PyTorch、PyPI 和 GitHub 软件源。镜像运行阶段不需要下载模型。

## 运行参数

首次运行可以复制环境变量模板：

```bash
cd /home/ub/project/detect_box_ws
cp docker/.env.example docker/.env
```

默认内容：

```text
DETECT_BOX_SIM_IMAGE=detect-box-sim:local
ROS_DOMAIN_ID=0
ROS_LOCALHOST_ONLY=0
RMW_IMPLEMENTATION=rmw_fastrtps_cpp
FASTDDS_BUILTIN_TRANSPORTS=UDPv4
```

`ROS_DOMAIN_ID` 必须与仿真相机发布节点和服务调用方一致。镜像不固定当前机器的 Domain ID。

默认保持 ROS 2 Humble 的 Fast DDS。容器内设置 `FASTDDS_BUILTIN_TRANSPORTS=UDPv4`，禁用同机优先的共享内存传输，使宿主机与容器通过 UDP 通信；宿主机其他模块无需修改自己的 DDS 配置。

## 启动与停止

```bash
./scripts/start_sim_detector.sh
./scripts/logs_sim_detector.sh
./scripts/stop_sim_detector.sh
```

也可以直接使用 Compose：

```bash
docker compose -f docker/compose.sim.yaml up -d
docker compose -f docker/compose.sim.yaml logs -f detector
docker compose -f docker/compose.sim.yaml down
```

Compose 使用宿主网络，使容器中的 DDS 能发现宿主机或同网段的 ROS 2 节点；GPU 通过 `gpus: all` 传入容器。

调试文件保存在宿主工作空间：

```text
debug_box_position_sim/
```

## 后续拉取预构建镜像

镜像上传到仓库后，在 `docker/.env` 中设置完整镜像地址，例如：

```text
DETECT_BOX_SIM_IMAGE=ghcr.io/<账号>/detect-box-sim:v0.1.0
```

同事执行：

```bash
./scripts/pull_sim_image.sh
./scripts/start_sim_detector.sh
```

上传镜像、仓库权限和版本发布在本地镜像验证通过后再配置。

## 服务验证

确保仿真持续发布：

```text
/camera/color/image_raw                       rgb8
/camera/color/camera_info
/camera/aligned_depth_to_color/image_raw      32FC1, m
```

调用服务：

```bash
ros2 service call /detect upper_limb_interface/srv/DetectAprilTag \
  "{capture_once: true}"
```

服务收到请求后等待请求之后发布的一组新同步帧。若在 `rgbd_wait_timeout_sec` 内没有收到，返回失败并打印三个输入话题。
