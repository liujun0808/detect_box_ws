# ROS 节点开机自启动说明

本目录用于管理 Detect Box 工作空间的 ROS 节点开机自启动。

## 文件说明

- `detect_box_ws.service`：systemd 服务文件。用于告诉 Linux 开机后如何启动本工作空间。
- `start_ros_nodes.sh`：实际启动脚本。负责加载 ROS Humble 和当前工作空间环境，然后启动 ROS 节点。
- `install_autostart.sh`：自启动安装脚本。负责把 service 文件复制到 systemd 目录、刷新 systemd、设置开机自启并重启服务。

## 自启动实现原理

开机启动链路如下：

```text
Linux 开机
  -> systemd
  -> /etc/systemd/system/detect_box_ws.service
  -> /home/user/project/detect_box_ws/scripts/start_ros_nodes.sh
  -> ROS 节点
```

其中 `detect_box_ws.service` 中的关键配置是：

```ini
ExecStart=/home/user/project/detect_box_ws/scripts/start_ros_nodes.sh
Restart=always
RestartSec=5
StartLimitIntervalSec=60
StartLimitBurst=5
WantedBy=multi-user.target
```

含义如下：

- `ExecStart`：指定 systemd 启动时要执行的脚本。
- `Restart=always`：如果脚本异常退出，systemd 会自动重新拉起。
- `RestartSec=5`：服务异常退出后，等待 5 秒再尝试重启。
- `StartLimitIntervalSec=60`：统计 60 秒内的启动失败次数。
- `StartLimitBurst=5`：60 秒内最多允许重启 5 次，超过后 systemd 暂停继续重启，服务进入 `failed` 状态。
- `WantedBy=multi-user.target`：表示该服务会在系统进入普通多用户模式时启动，也就是常见的开机自启动场景。

service 文件中还有：

```ini
ExecStartPre=/bin/sleep 5
```

这表示真正启动 ROS 节点前，先等待 5 秒。这样可以给系统、硬件设备、网络服务等留一点初始化时间，减少刚开机时的启动竞争问题。

## 当前启动内容

当前 `start_ros_nodes.sh` 的启动流程如下：

1. 加载 ROS Humble 环境：

   ```bash
   source /opt/ros/humble/setup.bash
   ```

2. 加载当前工作空间环境：

   ```bash
   source /home/user/project/detect_box_ws/install/setup.bash
   ```

3. 先启动 D435i 相机 launch：

   ```bash
   ros2 launch d435_publisher d435i_camera.launch.py &
   CAMERA_PID=$!
   ```

4. 等待 5 秒：

   ```bash
   sleep "${CAMERA_START_DELAY}"
   ```

5. 再启动检测服务节点：

   ```bash
   ros2 run detect_pkg detect_server_node &
   DETECT_PID=$!
   ```

6. 保持脚本运行，并监听主要 ROS 进程：

   ```bash
   wait -n "${CAMERA_PID}" "${DETECT_PID}"
   ```

如果相机 launch 或检测节点中的任意一个进程退出，`wait -n` 会返回，启动脚本也会退出。由于 systemd 服务配置了 `Restart=always`，systemd 会重新启动整个脚本。

同时，service 文件配置了重启限速：

```ini
StartLimitIntervalSec=60
StartLimitBurst=5
```

如果相机没插、节点启动失败或其他问题导致服务持续失败，systemd 会在 60 秒内最多重启 5 次。超过这个次数后，服务会进入 `failed` 状态，不再无限重启。

这通常不会影响系统其他程序启动，因为 `detect_box_ws.service` 是一个独立的 systemd 服务。失败后主要影响的是本 ROS 工作空间的节点，以及依赖这些节点的其他程序。

如果问题修复后需要重新启动服务，执行：

```bash
sudo systemctl reset-failed detect_box_ws.service
sudo systemctl restart detect_box_ws.service
```

## 安装或更新开机自启动

在项目根目录执行：

```bash
cd /home/user/project/detect_box_ws
./scripts/install_autostart.sh
```

该脚本内部会执行：

```bash
sudo install -m 0644 scripts/detect_box_ws.service /etc/systemd/system/detect_box_ws.service
sudo systemctl daemon-reload
sudo systemctl enable detect_box_ws.service
sudo systemctl restart detect_box_ws.service
```

各步骤含义：

- `install`：把项目中的 service 文件安装到 systemd 系统目录。
- `daemon-reload`：让 systemd 重新读取 service 配置。
- `enable`：设置为开机自启动。
- `restart`：立即重启该服务，让当前配置马上生效。

如果修改了 `detect_box_ws.service`，需要重新执行：

```bash
./scripts/install_autostart.sh
```

这是因为系统实际读取的是：

```text
/etc/systemd/system/detect_box_ws.service
```

而不是项目目录中的：

```text
/home/user/project/detect_box_ws/scripts/detect_box_ws.service
```

重新执行安装脚本后，会把项目中的新版 service 文件复制到 systemd 目录，并执行 `systemctl daemon-reload` 和 `systemctl restart`。

如果只修改了 `start_ros_nodes.sh`，通常不需要重新安装 service，只需要重启服务：

```bash
sudo systemctl restart detect_box_ws.service
```

## 查看状态和日志

查看是否已经设置为开机自启动：

```bash
systemctl is-enabled detect_box_ws.service
```

期望输出：

```text
enabled
```

查看当前运行状态：

```bash
systemctl status detect_box_ws.service
```

实时查看日志：

```bash
journalctl -u detect_box_ws.service -f
```

查看本次开机以来的日志：

```bash
journalctl -u detect_box_ws.service -b
```

## 添加新的 ROS 节点

需要修改：

```text
/home/user/project/detect_box_ws/scripts/start_ros_nodes.sh
```

假设要新增一个节点：

```bash
ros2 run example_pkg example_node
```

可以在已有启动命令后面添加：

```bash
ros2 run example_pkg example_node &
EXAMPLE_PID=$!
```

然后把新的 PID 加到最后的 `wait -n` 中：

```bash
wait -n "${CAMERA_PID}" "${DETECT_PID}" "${EXAMPLE_PID}"
```

同时还要在 `cleanup()` 函数中增加停止逻辑：

```bash
if [[ -n "${EXAMPLE_PID:-}" ]]; then
  kill "${EXAMPLE_PID}" 2>/dev/null || true
fi
```

修改完成后，先检查脚本语法：

```bash
bash -n scripts/start_ros_nodes.sh
```

然后重启服务：

```bash
sudo systemctl restart detect_box_ws.service
```

## 添加新的 launch 文件

添加 launch 文件的方式和添加节点类似。

例如要新增：

```bash
ros2 launch another_pkg another.launch.py
```

可以写成：

```bash
ros2 launch another_pkg another.launch.py &
ANOTHER_PID=$!
```

然后同样把 `ANOTHER_PID` 加到：

- `cleanup()` 函数中
- 最后的 `wait -n` 命令中

修改后执行：

```bash
bash -n scripts/start_ros_nodes.sh
sudo systemctl restart detect_box_ws.service
```

## 删除某个 ROS 节点

如果要从开机自启动中删除某个节点，需要在 `start_ros_nodes.sh` 中做这些修改：

1. 删除或注释该节点对应的 `ros2 run` 或 `ros2 launch` 命令。
2. 删除或注释该节点对应的 `*_PID=$!` 行。
3. 从 `cleanup()` 函数中删除该 PID 的停止逻辑。
4. 从最后的 `wait -n` 命令中删除该 PID。
5. 重启服务：

   ```bash
   sudo systemctl restart detect_box_ws.service
   ```

## 调整启动顺序

启动顺序由 `start_ros_nodes.sh` 中命令的先后顺序决定。

当前顺序是：

```text
D435i 相机 launch
  -> 等待 5 秒
  -> detect_server_node
```

如果要调整相机启动后等待检测节点启动的时间，修改：

```bash
CAMERA_START_DELAY=5
```

例如改成等待 10 秒：

```bash
CAMERA_START_DELAY=10
```

修改后重启服务：

```bash
sudo systemctl restart detect_box_ws.service
```

## 临时关闭开机自启动

如果只是想临时停止当前正在运行的 ROS 节点，但下次开机仍然自动启动，执行：

```bash
sudo systemctl stop detect_box_ws.service
```

这种方式只会停止当前服务，不会取消开机自启动。执行后检查自启动状态：

```bash
systemctl is-enabled detect_box_ws.service
```

如果输出仍然是：

```text
enabled
```

说明下次开机还会自动启动。

如果想临时关闭下次开机自启动，但保留 service 配置文件，执行：

```bash
sudo systemctl disable detect_box_ws.service
```

如果还想同时停止当前正在运行的 ROS 节点，执行：

```bash
sudo systemctl disable detect_box_ws.service
sudo systemctl stop detect_box_ws.service
```

这不会删除 `/etc/systemd/system/detect_box_ws.service`，只是取消开机自动启动。后续需要恢复时，执行：

```bash
sudo systemctl enable detect_box_ws.service
sudo systemctl start detect_box_ws.service
```

恢复后可以检查：

```bash
systemctl is-enabled detect_box_ws.service
systemctl status detect_box_ws.service
```

## 禁用或移除开机自启动

禁用开机自启动：

```bash
sudo systemctl disable detect_box_ws.service
```

停止当前正在运行的服务：

```bash
sudo systemctl stop detect_box_ws.service
```

如果要彻底删除已安装到 systemd 的 service 文件：

```bash
sudo rm /etc/systemd/system/detect_box_ws.service
sudo systemctl daemon-reload
```

## 常见检查项

如果服务启动失败，可以优先检查：

1. 工作空间是否已经编译：

   ```bash
   test -f /home/user/project/detect_box_ws/install/setup.bash
   ```

2. ROS Humble 是否存在：

   ```bash
   test -f /opt/ros/humble/setup.bash
   ```

3. 节点可执行名是否正确：

   ```bash
   source /opt/ros/humble/setup.bash
   source /home/user/project/detect_box_ws/install/setup.bash
   ros2 run detect_pkg detect_server_node
   ```

4. 查看 systemd 日志：

   ```bash
   journalctl -u detect_box_ws.service -b
   ```

5. 如果日志中出现类似错误：

   ```text
   /opt/ros/humble/setup.bash: line 8: AMENT_TRACE_SETUP_FILES: unbound variable
   ```

   说明启动脚本在加载 ROS 环境时启用了 `set -u`，而 ROS setup 脚本内部会访问未定义变量。当前 `start_ros_nodes.sh` 已经在 source ROS 环境时临时关闭了 `set -u`：

   ```bash
   set +u
   source "${ROS_SETUP}"
   source "${WS_SETUP}"
   set -u
   ```

   如果服务因为连续失败已经进入 `failed` 状态，修复后需要执行：

   ```bash
   sudo systemctl reset-failed detect_box_ws.service
   sudo systemctl restart detect_box_ws.service
   ```
