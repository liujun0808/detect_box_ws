# 检测节点开机自启动

该目录使用 systemd 管理 `/home/user/detect_box_ws` 中的检测服务节点。

自启只运行：

```bash
ros2 run detect_pkg detect_server_node
```

不再启动 `d435i_camera.launch.py` 或 `realsense2_camera`。检测节点空闲时不占用相机；收到 `detect` 服务请求后，节点才通过 librealsense2 打开 D435I、丢弃预热帧、完成检测并关闭相机。

## 文件

- `detect_box_ws.service`：systemd unit。
- `start_ros_nodes.sh`：加载 ROS 环境并运行检测节点。
- `install_autostart.sh`：安装 unit、刷新 systemd、启用并启动服务。

启动链路：

```text
systemd
  -> /home/user/detect_box_ws/scripts/start_ros_nodes.sh
  -> ros2 run detect_pkg detect_server_node
```

## 首次安装或本次更新后

本次同时修改了 service 文件和脚本路径，并且旧服务已经关闭，因此需要重新安装：

```bash
cd /home/user/detect_box_ws
chmod +x scripts/start_ros_nodes.sh scripts/install_autostart.sh
./scripts/install_autostart.sh
```

安装脚本会依次执行：

```bash
sudo install -m 0644 scripts/detect_box_ws.service /etc/systemd/system/detect_box_ws.service
sudo systemctl daemon-reload
sudo systemctl enable detect_box_ws.service
sudo systemctl restart detect_box_ws.service
```

因此不需要重启电脑。脚本执行完成后，新配置立即生效，并在以后开机时自动启动。

如果只修改了 `start_ros_nodes.sh`，无需重新安装 unit，只需：

```bash
sudo systemctl restart detect_box_ws.service
```

如果修改了 `detect_box_ws.service` 或安装路径，则应重新执行 `install_autostart.sh`。

## 构建代码后

检测代码修改并重新 `colcon build` 后，一般只需重启服务：

```bash
cd /home/user/detect_box_ws
colcon build --packages-select detect_pkg --symlink-install
sudo systemctl restart detect_box_ws.service
```

启动脚本每次运行都会加载：

```bash
source /opt/ros/humble/setup.bash
source /home/user/detect_box_ws/install/setup.bash
```

## 状态与日志

确认开机自启已启用：

```bash
systemctl is-enabled detect_box_ws.service
```

查看状态：

```bash
systemctl status detect_box_ws.service
```

实时日志：

```bash
journalctl -u detect_box_ws.service -f
```

本次开机日志：

```bash
journalctl -u detect_box_ws.service -b
```

停止并暂时保留开机自启：

```bash
sudo systemctl stop detect_box_ws.service
```

停止并取消开机自启：

```bash
sudo systemctl disable --now detect_box_ws.service
```

恢复启动：

```bash
sudo systemctl enable --now detect_box_ws.service
```

服务持续失败并进入 `failed` 状态时：

```bash
sudo systemctl reset-failed detect_box_ws.service
sudo systemctl restart detect_box_ws.service
```

## 注意

- 不要再单独自启相机节点，否则它会和检测节点在收到请求时争用 D435I。
- systemd 使用 `ROS_DOMAIN_ID=19`、CycloneDDS 和 `/home/user/dds/cyclonedds.xml`；调用端必须使用兼容的 ROS 2/DDS 配置。
- `Restart=always` 会在检测节点异常退出后等待 5 秒重新启动。
- unit 启动前会等待 5 秒，给系统和 USB 设备留出初始化时间。
