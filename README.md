# detect_box_ws

`detect_box_ws` 是基于 ROS 2 Humble 的箱体检测与定位工作空间。项目维护两套检测实现：

- `detect_pkg`：实体 D435 相机版本，通过 RealSense SDK 直接获取 RGB-D；
- `detect_pkg_sim`：仿真版本，通过 ROS 2 RGB-D 话题获取图像，使用 `/detect` 服务触发一次检测与定位。

当前 Docker 镜像只发布 `detect_pkg_sim`，目标平台为 x86_64 NVIDIA GPU 主机。实体相机版本不包含在仿真镜像内。

## 功能概览

仿真检测服务订阅同步 RGB-D 数据，在每次 `/detect` 请求后获取一组新帧，执行 YOLO-World 检测、候选区域点云处理和固定尺寸箱体中心估计，并通过原有服务接口返回 `base_link` 坐标系下的位置。

输入话题约定：

```text
/camera/color/image_raw                  sensor_msgs/Image, rgb8
/camera/color/camera_info                 sensor_msgs/CameraInfo
/camera/aligned_depth_to_color/image_raw  sensor_msgs/Image, 32FC1, m
```

服务接口保持不变：

```text
/detect
upper_limb_interface/srv/DetectAprilTag
```

## 目录结构

```text
detect_box_ws/
├── src/
│   ├── upper_limb_interface/             # DetectAprilTag.srv 服务定义
│   ├── detect_pkg/                       # 实体 D435 检测包，不进入仿真镜像
│   └── detect_pkg_sim/                   # 仿真检测包
│       ├── detect_box_sim_pipeline/      # Python 检测、点云和估计实现
│       ├── config/                       # 默认 ROS 2 参数 YAML
│       └── launch/                       # 仿真服务 launch 文件
├── models/
│   └── yolov8s-worldv2.pt                # YOLO-World 权重，构建镜像时复制
├── weights/
│   └── clip/ViT-B-32.pt                  # CLIP 权重，构建镜像时复制
├── docker/
│   ├── Dockerfile.sim                    # 仿真镜像定义
│   ├── compose.sim.build.yaml            # 本机构建 Compose 覆盖配置
│   ├── compose.sim.yaml                  # 工作空间内联调运行配置
│   └── requirements-sim.txt              # 镜像内 Python 依赖
├── scripts/
│   ├── build_sim_image.sh                # 构建本地仿真镜像
│   ├── sim_compose.sh                    # 工作空间内 Compose 包装器
│   ├── start_sim_detector.sh             # 本机启动仿真检测容器
│   ├── stop_sim_detector.sh              # 本机停止仿真检测容器
│   └── logs_sim_detector.sh              # 查看本机容器日志
└── deploy/detect_pkg_sim/                # 面向使用者的独立部署文件包
```

构建产物 `build/`、`install/`、`log/`，本机调试快照和模型权重均不应作为源码提交。

## 源码构建与测试

维护者在修改 ROS 接口或仿真包代码后，可先进行工作空间构建：

```bash
cd /home/ub/project/detect_box_ws
source /opt/ros/humble/setup.bash

colcon build \
  --packages-select upper_limb_interface detect_pkg_sim \
  --symlink-install
```

运行仿真包测试：

```bash
source install/setup.bash
colcon test --packages-select detect_pkg_sim
colcon test-result --all
```

本机直接运行 Python 节点时，需要满足 `detect_pkg_sim` 的 ROS 2、OpenCV、NumPy、SciPy、Ultralytics 和 CLIP 依赖；Docker 构建与部署镜像已封装这些依赖。

## 构建仿真镜像

### 前置条件

- Docker Engine 与 Docker Compose v2 可用；
- 当前用户有 Docker socket 权限，`docker version` 可正常访问 Server；
- 两个权重文件存在且保持原始内容：

```bash
ls -lh models/yolov8s-worldv2.pt
ls -lh weights/clip/ViT-B-32.pt
```

首次构建需要访问 Ubuntu、ROS、PyTorch、PyPI 与 GitHub 软件源。Dockerfile 会校验 YOLO 和 CLIP 权重的 SHA256，权重不匹配时构建会失败。

### 构建命令

```bash
cd /home/ub/project/detect_box_ws
./scripts/build_sim_image.sh
```

成功后产生本地标签：

```text
detect-box-sim:local
```

验证镜像：

```bash
docker image inspect detect-box-sim:local

docker run --rm --gpus all \
  --entrypoint /usr/bin/python3 \
  detect-box-sim:local \
  -c "import torch; print(torch.cuda.is_available()); print(torch.cuda.get_device_name(0))"
```

首次构建会下载 ROS 系统依赖、CUDA PyTorch 与 Python 包，并导出包含 CLIP 权重的镜像层，因此耗时较长。后续修改算法时，Docker 会复用未变化的依赖层。

## 镜像版本发布

每次可交付变更都使用明确版本标签，不覆盖已发布版本。以 `v0.1.0` 为例：

```bash
docker login registry.cn-hangzhou.aliyuncs.com

docker tag detect-box-sim:local \
  registry.cn-hangzhou.aliyuncs.com/keno/qi-carry-box-sim:v0.1.0

docker push registry.cn-hangzhou.aliyuncs.com/keno/qi-carry-box-sim:v0.1.0
```

后续算法、依赖、服务接口或模型权重更新时，重新构建并递增版本，例如：

```bash
./scripts/build_sim_image.sh

docker tag detect-box-sim:local \
  registry.cn-hangzhou.aliyuncs.com/keno/qi-carry-box-sim:v0.1.1

docker push registry.cn-hangzhou.aliyuncs.com/keno/qi-carry-box-sim:v0.1.1
```

`detect-box-sim:local` 是本机构建入口标签；`registry.cn-hangzhou.aliyuncs.com/keno/qi-carry-box-sim:vX.Y.Z` 是同一镜像用于阿里云发布的版本标签。重新构建只会更新本地标签，不会覆盖此前的发布标签。

`build_sim_image.sh` 会关闭 BuildKit 默认的 provenance/SBOM 证明清单，以兼容阿里云镜像仓库；这不影响镜像内的运行依赖或检测行为。

如果已有可用镜像，不需要为清单兼容问题重新安装依赖或编译源码。可使用一个只有 `FROM` 的临时 Dockerfile 复用全部镜像层，并生成不含 provenance/SBOM 的阿里云兼容清单：

```bash
SOURCE_IMAGE=liujun0808/detect_pkg_sim:v0.1.0
TARGET_IMAGE=registry.cn-hangzhou.aliyuncs.com/keno/qi-carry-box-sim:v0.1.0

printf 'FROM %s\n' "$SOURCE_IMAGE" \
  | docker buildx build \
      --platform linux/amd64 \
      --provenance=false \
      --sbom=false \
      --push \
      -t "$TARGET_IMAGE" \
      -f - \
      .
```

若普通 `docker push` 在所有层均显示 `Pushed` 后出现 `unknown manifest class for application/vnd.oci.empty.v1+json`，表示文件系统层已上传，但最终镜像清单未发布；应使用上面的命令重写清单。详细说明见部署包 README。

发布前应完成至少以下检查：

```bash
docker image inspect detect-box-sim:local
git diff --check
```

当外部参数 YAML 的格式或含义变化时，应同步更新 `deploy/detect_pkg_sim/config/box_position_estimation_sim.yaml`，并在部署说明中标明与镜像版本的对应关系。

## 本机仿真联调

本机已有仿真相机话题时，可使用 `scripts/` 下的脚本启动和查看工作空间内的检测容器。容器默认使用 Fast DDS，并通过 `FASTDDS_BUILTIN_TRANSPORTS=UDPv4` 避开 Docker 边界上的共享内存传输问题；宿主机调用端不需要切换 DDS 实现。

完整的同事部署、NVIDIA Container Toolkit 安装、镜像拉取、运行参数、外部 YAML 调参、服务调用与故障排查说明，请阅读：

[deploy/detect_pkg_sim/README.md](/home/ub/project/detect_box_ws/deploy/detect_pkg_sim/README.md)
