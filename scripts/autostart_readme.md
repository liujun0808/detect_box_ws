# 检测节点开机自启动

该目录使用 systemd 管理当前工作空间 /home/user/project/detect_box_ws 中的 ROS 2 检测节点。

自启启动的是：

~~~bash
ros2 launch detect_pkg box_position_estimation.launch.py
~~~

launch 文件会自动加载：

~~~text
src/detect_pkg/config/box_position_estimation.yaml
~~~

不再启动 d435i_camera.launch.py 或 realsense2_camera。检测节点空闲时不打开相机；收到 /detect 请求后，节点通过 librealsense2 获取 D435 RGB-D。由于当前配置 camera_keep_running: true，相机首次请求启动后会保持运行，直到检测节点停止。

## 文件

- detect_box_ws.service：systemd unit。
- start_ros_nodes.sh：加载 ROS 2 和工作空间环境，运行 launch。
- install_autostart.sh：安装 unit、刷新 systemd、启用并启动服务。

启动链路：

~~~text
systemd
  -> /home/user/project/detect_box_ws/scripts/start_ros_nodes.sh
  -> ros2 launch detect_pkg box_position_estimation.launch.py
  -> detect_server_node
~~~

detect_server_node 的 ROS 2 可执行入口会使用 py310 环境中的 Python：

~~~text
/home/user/miniconda3/envs/py310/bin/python
~~~

## 前置条件

先完成一次构建，并确认以下文件存在：

~~~bash
test -f /home/user/project/detect_box_ws/install/setup.bash
test -x /home/user/project/detect_box_ws/src/detect_pkg/scripts/detect_server_node
ls -lh /home/user/project/detect_box_ws/models/yolov8s-worldv2.pt
~~~

如果 Python 依赖尚未安装，在 py310 环境中安装：

~~~bash
conda activate py310
pip install numpy scipy opencv-python pyrealsense2 ultralytics
~~~

PyTorch 需要安装与当前显卡驱动匹配的 CUDA 版本。

## 首次安装或更新 service 后

~~~bash
cd /home/user/project/detect_box_ws
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
cd /home//project/detect_box_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select detect_pkg --symlink-install
sudo systemctl restart detect_box_ws.service
~~~

如果同时修改了 upper_limb_interface：

~~~bash
colcon build --packages-select upper_limb_interface detect_pkg --symlink-install
sudo systemctl restart detect_box_ws.service
~~~

启动脚本每次都会重新加载：

~~~bash
source /opt/ros/humble/setup.bash
source /home/user/project/detect_box_ws/install/setup.bash
~~~

## 手动启动验证

排查自启前，可以先停止 systemd 服务，再在终端运行：

~~~bash
sudo systemctl stop detect_box_ws.service
cd /home/user/project/detect_box_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch detect_pkg box_position_estimation.launch.py
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
source /home/user/project/detect_box_ws/install/setup.bash
ros2 service list | grep detect
~~~

触发一次检测：

~~~bash
ros2 service call /detect upper_limb_interface/srv/DetectAprilTag \
  "{capture_once: true}"
~~~

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
- 当前不强制设置外部 CycloneDDS 配置文件，避免引用旧用户目录下不存在的 DDS 配置文件。
- ROS_LOCALHOST_ONLY=0，允许局域网内符合 ROS_DOMAIN_ID 的节点通信。
- Restart=always 会在检测节点异常退出后等待 5 秒重启。
- unit 启动前等待 5 秒，为系统和 USB 设备初始化留出时间。
