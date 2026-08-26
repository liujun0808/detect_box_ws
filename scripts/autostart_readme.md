# 检测节点开机自启动

该目录使用 systemd 管理当前工作空间 /home/user/project/detect_box_pointcloud/detect_box_ws 中的 ROS 2 检测节点。

自启启动的是：

~~~bash
ros2 launch detect_pkg box_position_estimation.launch.py
~~~

launch 文件会自动加载：

~~~text
src/detect_pkg/config/box_position_estimation.yaml
~~~

服务不再启动 d435i_camera.launch.py 或 realsense2_camera。节点启动时会自动加载 YOLO-World、CLIP，并执行一次预热推理；因此首次 `/detect` 请求不再承担模型初始化耗时。RealSense 相机仍保持按请求打开：节点空闲时不占用相机，收到 `/detect` 请求后通过 librealsense2 获取 D435 RGB-D。由于当前配置 `camera_keep_running: true`，相机首次请求启动后会保持运行，直到检测节点停止。

## 文件

- detect_box_ws.service：systemd unit。
- start_ros_nodes.sh：加载 ROS 2 和工作空间环境，运行 launch。
- install_autostart.sh：安装 unit、刷新 systemd、启用并启动服务。
- `models/yolov8s-worldv2.pt`：YOLO-World 检测模型。
- `weights/clip/ViT-B-32.pt`：CLIP 文本类别编码权重。

启动链路：

~~~text
systemd
  -> /home/user/project/detect_box_pointcloud/detect_box_ws/scripts/start_ros_nodes.sh
  -> ros2 launch detect_pkg box_position_estimation.launch.py
  -> detect_server_node
~~~

detect_server_node 的 ROS 2 可执行入口会使用 py310 环境中的 Python：

~~~text
/home/user/miniconda3/envs/py310/bin/python
~~~

systemd 不依赖交互式 `conda activate`。`start_ros_nodes.sh` 和
`detect_server_node` 会直接使用上面的绝对 Python 路径。

## 前置条件

先完成一次构建，并确认以下文件存在：

~~~bash
test -f /home/user/project/detect_box_pointcloud/detect_box_ws/install/setup.bash
test -x /home/user/project/detect_box_pointcloud/detect_box_ws/src/detect_pkg/scripts/detect_server_node
ls -lh /home/user/project/detect_box_pointcloud/detect_box_ws/models/yolov8s-worldv2.pt
ls -lh /home/user/project/detect_box_pointcloud/detect_box_ws/weights/clip/ViT-B-32.pt
~~~

如果 Python 依赖尚未安装，在 py310 环境中安装：

~~~bash
conda activate py310
python -m pip install numpy scipy opencv-python pyrealsense2 ultralytics
python -m pip install "git+https://github.com/ultralytics/CLIP.git"
~~~

PyTorch 需要安装与当前显卡驱动匹配的 CUDA 版本。

## 启动阶段模型预加载

配置文件中的以下参数控制启动预热：

~~~yaml
yolo_startup_warmup: true
~~~

启动日志正常时应依次出现：

~~~text
Loading YOLO-World and CLIP model at startup...
YOLO-World startup initialization complete
New box-position service scaffold ready
~~~

如果模型、CLIP 包或 CLIP 权重缺失，节点会在启动阶段报错，不会等到第一次服务请求才报错。

## 首次安装或更新 service 后

~~~bash
cd /home/user/project/detect_box_pointcloud/detect_box_ws
chmod +x scripts/start_ros_nodes.sh scripts/install_autostart.sh
./scripts/install_autostart.sh
~~~

安装完成后不需要重启电脑，服务会立即启动，并在以后开机时自动启动。

如果只修改了 start_ros_nodes.sh，无需重新安装 unit：

~~~bash
sudo systemctl restart detect_box_ws.service
~~~

如果修改了 detect_box_ws.service、User 或工作空间路径，应重新执行：

~~~bash
./scripts/install_autostart.sh
~~~

## 重新编译代码后

~~~bash
cd /home/user/project/detect_box_pointcloud/detect_box_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select detect_pkg --symlink-install
sudo systemctl restart detect_box_ws.service
~~~

如果同时修改了 upper_limb_interface：

~~~bash
colcon build --packages-select upper_limb_interface detect_pkg --symlink-install
sudo systemctl restart detect_box_ws.service
~~~

如果修改了模型、CLIP 权重或 Python 依赖，通常不需要重新编译，只需重启服务：

~~~bash
sudo systemctl restart detect_box_ws.service
~~~

启动脚本每次都会重新加载：

~~~bash
source /opt/ros/humble/setup.bash
source /home/user/project/detect_box_pointcloud/detect_box_ws/install/setup.bash
~~~

## 手动启动验证

排查自启前，可以先停止 systemd 服务，再在终端运行：

~~~bash
sudo systemctl stop detect_box_ws.service
cd /home/user/project/detect_box_pointcloud/detect_box_ws
./scripts/start_ros_nodes.sh
~~~

确认 launch 正常后，按 Ctrl-C 退出，再恢复：

~~~bash
sudo systemctl start detect_box_ws.service
~~~

## 状态与日志

确认开机自启已启用：

~~~bash
systemctl is-enabled detect_box_ws.service
~~~

查看状态：

~~~bash
systemctl status detect_box_ws.service
~~~

实时查看日志：

~~~bash
journalctl -u detect_box_ws.service -f
~~~

查看本次开机日志：

~~~bash
journalctl -u detect_box_ws.service -b
~~~

检测节点启动后，可在另一个终端检查服务：

~~~bash
source /opt/ros/humble/setup.bash
source /home/user/project/detect_box_pointcloud/detect_box_ws/install/setup.bash
ros2 service list | grep detect
~~~

触发一次检测：

~~~bash
ros2 service call /detect upper_limb_interface/srv/DetectAprilTag \
  "{capture_once: true}"
~~~

## 调试快照

调试快照目录为：

~~~text
/home/user/project/detect_box_pointcloud/detect_box_ws/debug_box_position
~~~

配置 `debug_max_snapshots: 10` 时，程序按请求目录滚动保留最近 10 次数据。失败检测也会保存：

~~~text
color_raw.png
~~~

成功检测后还会保存带框图和点云文件（如果对应数据已生成）。

## 控制服务

停止但保留开机自启：

~~~bash
sudo systemctl stop detect_box_ws.service
~~~

停止并取消开机自启：

~~~bash
sudo systemctl disable --now detect_box_ws.service
~~~

恢复开机自启并立即启动：

~~~bash
sudo systemctl enable --now detect_box_ws.service
~~~

服务持续失败时：

~~~bash
sudo systemctl reset-failed detect_box_ws.service
sudo systemctl restart detect_box_ws.service
~~~

## ROS 通信注意事项

- 不要再单独启动相机节点，否则可能与检测节点争用 D435。
- 当前脚本设置 ROS_DOMAIN_ID=22；调用 /detect 的其他 ROS 2 终端需要使用相同的 ROS_DOMAIN_ID。
- systemd unit 使用 `User=user`、工作目录 `/home/user/project/detect_box_pointcloud/detect_box_ws`，并设置 `HOME=/home/user`。
- systemd unit 显式使用 `ROS_DOMAIN_ID=22`、`RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` 和 `/home/user/dds/cyclonedds.xml`，以保证自启节点与交互式终端使用相同的 ROS 2 DDS 配置。
- 当前不强制设置外部 CycloneDDS 配置文件，避免引用旧用户目录下不存在的 DDS 配置文件。
- ROS_LOCALHOST_ONLY=0，允许局域网内符合 ROS_DOMAIN_ID 的节点通信。
- Restart=always 会在检测节点异常退出后等待 5 秒重启。
- unit 启动前等待 5 秒，为系统和 USB 设备初始化留出时间。
