# Jetson Orin NX + Conda + YOLOE-26 部署与测试手册

本文是当前 `detect_box_ws` 的唯一推荐部署路径。目标设备为 Jetson Orin NX 16 GB、JetPack 6.2.1（Jetson Linux R36.4.4、CUDA 12.6）以及 ROS 2 Humble。

当前方案的边界：

- 使用 `yoloe-26s-seg.pt`，通过英文文字提示词做实例分割；
- mask 用于筛选深度像素并生成点云，要求覆盖整个箱体实例；
- 最终只返回箱体中心 `box_pose`，姿态字段仍为单位四元数；
- 外参方向是 `camera -> base_link`，矩阵就是配置文件中的 `camera_to_base_row_major`；
- 当前只运行 PyTorch，不导出 TensorRT FP16 engine；
- Conda 环境名固定为 `detect_box`，默认安装目录固定为 `/home/user/miniforge3`；如果实际路径不同，只需按文中的“路径替换”说明修改。
- ROS 工作空间编译使用系统 `/opt/ros/humble`，节点启动器直接调用 Conda Python 绝对路径，不需要 `conda activate`。

Ultralytics 官方说明：YOLOE-26 需要 Ultralytics 8.4.0 或更高版本；第一次 `set_classes()` 会下载 CLIP tokenizer 和约 254 MB 的 `mobileclip2_b.ts` 文本编码器。请在有网络时完成一次提示词测试，之后可以离线运行。

## 1. 清理旧的 venv 方案

以下命令只删除本工作区中此前为 venv 方案创建的内容，不删除系统 ROS、CUDA、TensorRT、RealSense 或用户源码。执行前确认当前目录确实是工作空间：

```bash
cd /home/user/liujun/detect_box_ws
pwd
```

如果确认无误，执行：

```bash
rm -rf .venv .local-cuda .ultralytics
rm -f models/yoloe-26s-seg.pt mobileclip2_b.ts
find src/detect_pkg/detect_box_pipeline -type d -name __pycache__ -prune -exec rm -rf {} +
```

检查旧环境已经不存在：

```bash
test ! -e .venv
test ! -e .local-cuda
test ! -e models/yoloe-26s-seg.pt
```

不要删除 `src/`、`build/`、`install/` 或 `log/` 中其他用户文件。如果后面遇到明显的旧构建缓存问题，只清理本包的 build/install 产物，不要使用 `rm -rf /`、`rm -rf ~` 或清空整个工作空间。

## 2. 确认 Jetson、ROS 和 CUDA

先不要创建 Python 环境，先确认系统版本：

```bash
uname -m
cat /etc/nv_tegra_release
cat /etc/os-release | grep -E '^(NAME|VERSION|VERSION_ID)='
python3 --version
nvcc --version | tail -5
ls /opt/ros
```

本项目目标输出应为：

```text
aarch64
R36 (release), REVISION: 4.4
Python 3.10.x
存在 /opt/ros/humble
CUDA release 12.6
```

加载 ROS 并确认接口包可见：

```bash
source /opt/ros/humble/setup.bash
ros2 --version
ros2 interface show upper_limb_interface/srv/DetectAprilTag
```

如果最后一条找不到接口，先解决 `upper_limb_interface` 的安装或工作空间 source 问题，不要进入 YOLOE 调试。

## 3. 安装系统依赖

ROS 2 Humble 已安装的情况下，只安装编译、Python 扩展和 RealSense 所需的系统包：

```bash
sudo apt update
sudo apt install -y \
  build-essential cmake git curl wget pkg-config \
  python3-dev python3-pip \
  python3-colcon-common-extensions python3-rosdep python3-vcstool \
  libopenblas-dev libusb-1.0-0-dev libudev-dev libssl-dev \
  udev libglib2.0-0 libgl1
```

确认 JetPack 自带的 CUDA、cuDNN、TensorRT 可以被系统发现：

```bash
ldconfig -p | grep -E 'libcudnn|libnvinfer|libcudart' | head
python3 - <<'PY'
try:
    import tensorrt
    print("TensorRT:", tensorrt.__version__)
except Exception as exc:
    print("TensorRT import failed:", exc)
PY
```

本阶段不需要安装 TensorRT engine，也不需要执行任何 `yolo export` 命令。

## 4. 安装 Miniforge（Conda）

Jetson 是 ARM64，下载 `Linux-aarch64` 安装器。已经有可用 Conda 的机器可以跳过下载，只需确认 `conda` 的根目录和环境路径与后文一致。

```bash
CONDA_ROOT=/home/user/miniforge3
INSTALLER=/tmp/Miniforge3-Linux-aarch64.sh

curl -fL \
  -o "${INSTALLER}" \
  https://github.com/conda-forge/miniforge/releases/latest/download/Miniforge3-Linux-aarch64.sh
bash "${INSTALLER}" -b -p "${CONDA_ROOT}"

source "${CONDA_ROOT}/etc/profile.d/conda.sh"
conda init bash
conda --version
```

重新打开一个终端，或手动加载：

```bash
source /home/user/miniforge3/etc/profile.d/conda.sh
```

如果公司网络无法访问 GitHub，可以使用已下载的 Miniforge 安装器；不要把 x86_64 安装器复制到 NX。

## 5. 创建 Conda 环境

Python 固定为 3.10，与 JetPack 6.2.1、ROS 2 Humble 和当前 Jetson PyTorch wheel 的 ABI 对齐：

```bash
source /home/user/miniforge3/etc/profile.d/conda.sh
conda create -n detect_box -c conda-forge \
  python=3.10 pip setuptools wheel \
  numpy=1.26 scipy=1.11 \
  opencv pillow pyyaml requests psutil matplotlib pandas \
  filelock typing_extensions sympy networkx jinja2 fsspec mpmath \
  cloudpickle polars -y
conda activate detect_box
python --version
python -c "import platform, sys; print(platform.machine()); print(sys.executable)"
```

应看到：

```text
Python 3.10.x
aarch64
/home/user/miniforge3/envs/detect_box/bin/python
```

后续所有 pip 命令都必须使用 `python -m pip`，并且必须在 `detect_box` 环境中执行。不要在这个环境中执行没有版本约束的 `pip install torch`。

## 6. 安装 Jetson CUDA 版 PyTorch

### 6.1 先确认 CUDA/JetPack 版本

```bash
cat /etc/nv_tegra_release
nvcc --version | grep release
```

### 6.2 推荐安装方式：Jetson AI Lab JP6/CUDA 12.6 索引

对于 JetPack 6.2/6.2.1，使用专门的 ARM64 CUDA 12.6 索引。当前推荐先装 PyTorch 2.8.0 和 torchvision 0.23.0；不要从普通 PyPI 拉取通用 Linux wheel。使用 `--index-url` 而不是 `--extra-index-url`，避免 pip 回退到不兼容的通用包：

```bash
conda activate detect_box
python -m pip install --no-cache-dir --no-deps \
  torch==2.8.0 torchvision==0.23.0 \
  --index-url https://pypi.jetson-ai-lab.io/jp6/cu126
```

如果该索引临时不可访问，先不要改装普通 PyPI 的 torch。改为查看 NVIDIA 的 [Jetson PyTorch 安装说明](https://docs.nvidia.com/deeplearning/frameworks/install-pytorch-jetson-platform/index.html)和[兼容性说明](https://docs.nvidia.com/deeplearning/frameworks/install-pytorch-jetson-platform-release-notes/pytorch-jetson-rel.html)，下载与 JetPack 6.2.1、Python 3.10、`linux_aarch64` 对应的 wheel，再使用：

```bash
python -m pip install --no-cache-dir --no-deps /absolute/path/to/torch-*.whl
python -m pip install --no-cache-dir --no-deps /absolute/path/to/torchvision-*.whl
```

### 6.3 验证 PyTorch CUDA

```bash
python - <<'PY'
import torch
print("torch:", torch.__version__)
print("torch.version.cuda:", torch.version.cuda)
print("cuda available:", torch.cuda.is_available())
if torch.cuda.is_available():
    print("device:", torch.cuda.get_device_name(0))
    x = torch.rand((1024, 1024), device="cuda")
    print("cuda tensor:", x.mean().item())
PY
```

期望 `cuda available: True`。如果 import 报 `libcudss.so.0`、`libcusparseLt.so` 或其他 CUDA 动态库缺失，先检查 JetPack 软件源中对应的系统包：

```bash
ldconfig -p | grep -E 'cudss|cusparseLt|cudnn' || true
apt-cache search cudss
apt-cache search cusparselt
```

只安装与 JetPack 6.2.1/CUDA 12.6 匹配的 NVIDIA 包，然后重复 6.3。不要把另一台机器的 `.so` 文件复制到工作空间，也不要使用此前 venv 方案的本地库目录。

如果出现 `NvRmMemInitNvmap failed`、`Memory Manager Not supported`，通常是当前 shell/容器没有暴露 Jetson GPU 设备节点；这不是通过重装 Ultralytics 可以解决的问题，应先在设备本机检查 `/dev/nvhost-*`、JetPack 驱动和 `tegrastats`。

## 7. 安装 Ultralytics、YOLOE tokenizer 和 RealSense Python 接口

先安装 Ultralytics 的纯 Python 包，但禁止它自动替换 Jetson PyTorch：

```bash
conda activate detect_box
python -m pip install --proxy http://127.0.0.1:17890 --no-cache-dir --no-deps 'ultralytics==8.4.144'
python -m pip install --proxy http://127.0.0.1:17890 --no-cache-dir --no-deps \
  git+https://github.com/ultralytics/CLIP.git
python -m pip install --proxy http://127.0.0.1:17890--no-cache-dir ftfy regex tqdm ultralytics-thop
```

验证基础 import：

```bash
python - <<'PY'
import cv2, numpy, scipy, torch, ultralytics
print("numpy:", numpy.__version__)
print("opencv:", cv2.__version__)
print("scipy:", scipy.__version__)
print("torch:", torch.__version__)
print("ultralytics:", ultralytics.__version__)
print("cuda:", torch.cuda.is_available())
PY
```

安装 RealSense Python binding：

```bash
python -m pip install --no-cache-dir pyrealsense2
python -c "import pyrealsense2 as rs; print('pyrealsense2:', rs.__file__)"
```

如果 ARM64 没有可用 pip wheel，先确认系统是否已经安装 Python binding：

```bash
python3 - <<'PY'
import pyrealsense2 as rs
print(rs.__file__)
PY
```

如果系统 Python 可以导入而 Conda Python 不可以，把实际目录加入当前 shell 的 `PYTHONPATH`，再测试：

```bash
export PYTHONPATH="/usr/lib/python3/dist-packages:/usr/local/lib/python3.10/dist-packages:${PYTHONPATH:-}"
python -c "import pyrealsense2 as rs; print(rs.__file__)"
```

如果仍然没有 binding，按照 RealSense 官方 Jetson 安装说明编译 `librealsense` 的 Python binding，编译时使用当前 Conda 解释器：

```bash
cmake .. -DCMAKE_BUILD_TYPE=Release \
  -DBUILD_EXAMPLES=false \
  -DBUILD_GRAPHICAL_EXAMPLES=false \
  -DBUILD_PYTHON_BINDINGS=true \
  -DPYTHON_EXECUTABLE="$(which python)"
```

## 8. 下载 YOLOE-26 权重和文本编码器

始终在工作空间根目录执行下面的命令。YOLOE-26 的文本编码器按当前工作目录查找；保持 `mobileclip2_b.ts` 位于工作空间根目录，可以避免每次启动重复下载。

```bash
cd /home/user/liujun/detect_box_ws
conda activate detect_box
mkdir -p models
```

先让 Ultralytics 从工作空间根目录下载 checkpoint，再把它移动到配置指定的 `models` 目录：

```bash
if [ ! -f models/yoloe-26s-seg.pt ]; then
  python - <<'PY'
from ultralytics import YOLOE
YOLOE('yoloe-26s-seg.pt')
PY
  mv yoloe-26s-seg.pt models/yoloe-26s-seg.pt
fi
```

然后执行一次文字提示和 mask 分支测试：

```bash
python - <<'PY'
import numpy as np
import torch
from ultralytics import YOLOE

model = YOLOE('/home/user/liujun/detect_box_ws/models/yoloe-26s-seg.pt')
prompts = [
    'green plastic crate',
    'plastic crate',
    'storage crate',
    'green storage box',
    'green box',
    'plastic container',
]
model.set_classes(prompts)

device = 'cuda:0' if torch.cuda.is_available() else 'cpu'
print('device:', device)
result = model.predict(
    source=np.zeros((320, 320, 3), dtype=np.uint8),
    imgsz=320,
    device=device,
    conf=0.1,
    retina_masks=True,
    verbose=False,
)
print('prompt test completed; boxes:', len(result[0].boxes))
PY
```

检查两个模型文件：

```bash
ls -lh /home/user/liujun/detect_box_ws/models/yoloe-26s-seg.pt
ls -lh /home/user/liujun/detect_box_ws/mobileclip2_b.ts
```

期望权重约几十 MB，`mobileclip2_b.ts` 约 254 MB。空白图没有检测框是正常的；这里主要验证 checkpoint、`set_classes()`、文本编码器、mask 分支和 CUDA 推理链路都能运行。

如果需要验证真实检测，可使用一张本地箱体图片：

```bash
python - <<'PY'
from ultralytics import YOLOE
model = YOLOE('/home/user/liujun/detect_box_ws/models/yoloe-26s-seg.pt')
model.set_classes(['green plastic crate', 'plastic crate', 'storage crate'])
results = model.predict(
    source='/absolute/path/to/crate.jpg',
    device='cuda:0',
    imgsz=640,
    conf=0.1,
    retina_masks=True,
    save=True,
)
print(results[0].boxes, results[0].masks)
PY
```

不要执行 `model.export()`；本阶段不使用 TensorRT engine。

## 9. 编译 ROS 工作空间

编译阶段使用系统 ROS/colcon，不需要激活 Conda。Conda 只提供节点运行时使用的 PyTorch、Ultralytics 和 RealSense Python 包。

每次从新终端开始，编译只需要加载 ROS：

```bash
source /opt/ros/humble/setup.bash
cd /home/user/liujun/detect_box_ws
```

如果需要单独确认 Conda 运行时能够导入 ROS Python 包，使用绝对解释器测试，不需要 `conda activate`：

```bash
PYTHONPATH="/opt/ros/humble/lib/python3.10/site-packages:/opt/ros/humble/local/lib/python3.10/dist-packages:${PYTHONPATH:-}" \
/home/user/miniforge3/envs/detect_box/bin/python - <<'PY'
import rclpy
from geometry_msgs.msg import Pose
print('rclpy:', rclpy.__file__)
print('geometry_msgs:', Pose)
PY
```

安装依赖并编译：

```bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select upper_limb_interface detect_pkg --symlink-install
source install/setup.bash
```

如果 `upper_limb_interface` 已经由系统安装，只需：

```bash
colcon build --packages-select detect_pkg --symlink-install
source install/setup.bash
```

编译后检查两个 ROS 包装器内置的运行时解释器路径：

```bash
grep DETECT_BOX_PYTHON src/detect_pkg/scripts/detect_server_node
grep DETECT_BOX_PYTHON scripts/start_ros_nodes.sh
```

默认路径应为：

```text
/home/user/miniforge3/envs/detect_box/bin/python
```

节点启动时不需要执行 `conda activate`。`src/detect_pkg/scripts/detect_server_node` 和
`scripts/start_ros_nodes.sh` 已经直接写入绝对解释器路径。如果 Conda 安装在其他目录，修改这两个文件中的 `DETECT_BOX_PYTHON` 默认值；不要把系统 Python 作为替代：

```bash
sed -i 's#^DETECT_BOX_PYTHON=.*#DETECT_BOX_PYTHON="${DETECT_BOX_PYTHON:-/absolute/path/to/miniforge3/envs/detect_box/bin/python}"#' \
  src/detect_pkg/scripts/detect_server_node
sed -i 's#^export DETECT_BOX_PYTHON=.*#export DETECT_BOX_PYTHON="${DETECT_BOX_PYTHON:-/absolute/path/to/miniforge3/envs/detect_box/bin/python}"#' \
  scripts/start_ros_nodes.sh
```

## 10. 运行单元测试和代码检查

单元测试和 ROS 编译测试也不需要激活 Conda：

```bash
source /opt/ros/humble/setup.bash
cd /home/user/liujun/detect_box_ws
/usr/bin/python3 -m pytest -q src/detect_pkg/test/test_mask_point_cloud.py
colcon test --packages-select detect_pkg --event-handlers console_direct+
colcon test-result --verbose
```

重点检查：

- mask 外的深度点不会进入 ROI 点云；
- mask 内的无效深度会被过滤；
- 可选腐蚀只影响点云采样，不修改保存的原始 mask；
- 最终仍返回 `box_pose`，没有引入新的 ROS 服务接口。

## 11. 启动节点和调用 `/detect`

推荐前台启动。这里的 `ros2 launch` 使用系统 ROS Python，但它启动的
`detect_server_node` 包装器会直接调用文件中指定的 Conda Python；不需要 `conda activate`：

```bash
source /opt/ros/humble/setup.bash
cd /home/user/liujun/detect_box_ws
source install/setup.bash
ros2 launch detect_pkg box_position_estimation.launch.py
```

另开终端调用：

```bash
source /opt/ros/humble/setup.bash
source /home/user/liujun/detect_box_ws/install/setup.bash
ros2 service call /detect upper_limb_interface/srv/DetectAprilTag \
  "{capture_once: true}"
```

第一次请求会包含 YOLOE 模型、MobileCLIP 和 CUDA 内核初始化。成功日志应包含类似：

```text
YOLOE actual inference device: cuda:0
```

服务响应中只把箱体中心写入 `box_pose`；orientation 仍是单位四元数。日志中的提示词类别、mask 像素数和质量指标用于排查，不改变服务接口。

## 12. 检查 mask 和点云

生产配置默认关闭调试输出。如果要检查 mask 是否覆盖整个箱体，临时修改：

```yaml
debug_enabled: true
debug_output_dir: /home/user/liujun/detect_box_ws/debug_box_position
```

重新启动节点并调用一次服务，然后查看最新目录：

```bash
find /home/user/liujun/detect_box_ws/debug_box_position \
  -maxdepth 2 -type f -printf '%TY-%Tm-%Td %TH:%TM %p\n' | sort | tail -20
```

应能看到：

```text
color_with_yoloe_mask.png
mask_roi_cloud.ply
```

其中：

- `color_with_yoloe_mask.png` 包含完整实例 mask 叠加、bbox、类别和置信度；
- `mask_roi_cloud.ply` 是 mask 与有效深度相交后反投影得到的原始点云；
- 调试阶段不再计算或保存单独 mask、原始彩色图和最终候选点云；
- 最终位置仍会通过当前 `camera -> base_link` 外参转换，服务仍只返回中心位置。

验证无误后恢复：

```yaml
debug_enabled: false
```

## 13. systemd 自启动

ROS 节点包装器已经默认使用：

```text
/home/user/miniforge3/envs/detect_box/bin/python
```

因此 systemd 不需要执行交互式 `conda activate`。它只需要通过 `scripts/start_ros_nodes.sh` source ROS，并用绝对路径启动 Conda Python。先确认脚本可执行：

```bash
cd /home/user/liujun/detect_box_ws
chmod +x scripts/start_ros_nodes.sh scripts/install_autostart.sh
```

如果 Conda 路径不是默认路径，在安装服务前设置：

```bash
export DETECT_BOX_PYTHON=/absolute/path/to/miniforge3/envs/detect_box/bin/python
```

并把同一个绝对路径写入 `scripts/start_ros_nodes.sh` 的默认值，随后执行：

```bash
./scripts/install_autostart.sh
sudo systemctl status detect_box_ws.service
journalctl -u detect_box_ws.service -f
```

systemd 日志中若出现 `Python executable not found`，先检查：

```bash
ls -l /home/user/miniforge3/envs/detect_box/bin/python
systemctl cat detect_box_ws.service
```

## 14. 故障定位顺序

### `No module named torch`

```bash
conda activate detect_box
which python
python -c "import torch; print(torch.__version__)"
```

必须指向 `/home/user/miniforge3/envs/detect_box/bin/python`，并重新安装 Jetson 专用 wheel。

### `torch.cuda.is_available()` 为 `False`

先看 JetPack、CUDA 动态库和 GPU 设备节点，再看 PyTorch wheel。不要先降低 YOLO 置信度，也不要把 YAML 的 `yolo_device` 改成 CPU 来掩盖安装问题。

```bash
tegrastats
ls -l /dev/nvhost-* 2>/dev/null | head
python -c "import torch; print(torch.__version__, torch.version.cuda, torch.cuda.is_available())"
```

### 每次 `set_classes()` 都重新下载 `mobileclip2_b.ts`

确认启动前的工作目录和文件：

```bash
cd /home/user/liujun/detect_box_ws
ls -lh mobileclip2_b.ts
```

systemd 已通过 `WorkingDirectory` 和 `start_ros_nodes.sh` 使用工作空间目录；手动运行 YOLOE 时也必须先 `cd` 到工作空间。

### YOLOE 找不到权重

```bash
ls -lh /home/user/liujun/detect_box_ws/models/yoloe-26s-seg.pt
grep yolo_model_path src/detect_pkg/config/box_position_estimation.yaml
```

两处应一致，且文件必须是 `yoloe-26s-seg.pt`，不要误用普通 `yolo26s-seg.pt`。

### mask 覆盖不完整

先打开 `debug_enabled`，检查 `color_with_yoloe_mask.png`。优先调整文字提示词和 `yolo_confidence_threshold`，不要先改点云优化器；点云阶段只能使用 mask 内的深度，不能恢复漏掉的实例区域。

### 点云太少或中心失败

按顺序检查：

1. mask 是否确实覆盖箱体；
2. 深度是否在 `depth_min_m`～`depth_max_m`；
3. `pointcloud_mask_erosion_px` 是否过大；
4. `pointcloud_min_points`、SOR 和聚类阈值是否适合现场深度噪声；
5. 最后再看内壁、上边沿和 3DoF 优化日志。

## 15. 固定配置确认清单

部署完成后，确认以下内容没有被 Conda/安装过程改回旧方案：

```text
模型：models/yoloe-26s-seg.pt
文本提示：yolo_class_prompts 中的六个箱体提示词
输出：只返回 box_pose 中心
外参方向：camera -> base_link
不导出 TensorRT engine
Python：/home/user/miniforge3/envs/detect_box/bin/python
```

当前外参矩阵为：

```text
 0.000000 -0.766044  0.642788  0.105590
-1.000000  0.000000  0.000000  0.032500
 0.000000 -0.642788 -0.766044  0.717260
 0.000000  0.000000  0.000000  1.000000
```

不要把它替换为逆矩阵，也不要同时把 D435 静态 TF 当成同一个方向重复应用。

## 16. 参考资料

- [NVIDIA Jetson PyTorch 安装说明](https://docs.nvidia.com/deeplearning/frameworks/install-pytorch-jetson-platform/index.html)
- [NVIDIA Jetson PyTorch 兼容性说明](https://docs.nvidia.com/deeplearning/frameworks/install-pytorch-jetson-platform-release-notes/pytorch-jetson-rel.html)
- [Ultralytics YOLOE](https://docs.ultralytics.com/models/yoloe)
- [JetPack 6.2.1 发布说明](https://docs.nvidia.com/jetson/jetpack/release-notes/index.html)
- [RealSense Python wrapper](https://github.com/IntelRealSense/librealsense/blob/master/wrappers/python/readme.md)

## 17. 性能调优顺序

节点每次请求都会输出一行 `Detection timing`，先根据阶段耗时调参，不要同时修改所有参数：

```text
capture, yoloe_segment, pointcloud_extract, pointcloud_process,
optimizer, pipeline, debug_snapshot, total
```

建议顺序：

1. `debug_snapshot` 较大：当前实现已经只保存 `color_with_yoloe_mask.png` 和 `mask_roi_cloud.ply`；生产环境可将 `debug_enabled` 设为 `false`。
2. `yoloe_segment` 较大：把 `yolo_image_size` 从 640 试到 512，确认 mask 仍覆盖整个箱体；必要时恢复 640。`yolo_max_detections` 可从 10 降到 3，因为当前服务只返回最高置信度的一个实例。
3. `capture` 较大：把 `camera_request_discard_frames` 从 5 试到 2；首次启动延迟仍高时，再把 `camera_warmup_frames` 从 30 试到 15，但必须检查曝光和深度稳定性。
4. `pointcloud_process` 较大：把 `inner_wall_ransac_iterations` 和 `top_edge_ransac_iterations` 从 300 试到 120；当前聚类邻域已经改为一次性批量查询。
5. `optimizer` 较大：把 `optimizer_representative_point_count` 从 4000 试到 2500，再把 `optimizer_max_iterations` 从 200 试到 100。最终质量检查仍使用完整候选点云。
6. 仍然超时才尝试把 `pointcloud_pixel_stride` 从 2 改为 3；这会明显减少点数，但可能降低内壁和上边沿拟合稳定性。

每次只改一组参数，连续请求至少测试 5 次，同时记录 `position_confidence`、`surface_rmse_m`、`surface_inlier_ratio` 和 mask 覆盖情况。
