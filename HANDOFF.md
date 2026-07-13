# HANDOFF

## 当前任务

工作区：`/home/user/detect_box_ws`，ROS 2 Humble，ARM64。

`detect_pkg/detect_server_node` 从 RealSense 彩色图中检测 AprilTag，根据 tag 在目标物上的安装位姿求 `T_camera_target`，再通过固定外参计算：

```text
T_base_target = T_base_camera * T_camera_target
```

结果通过 `detect` 服务返回，同时做位置范围检查并保存调试图。

主要文件：

- `src/detect_pkg/src/detect_sever_node.cpp`
- `src/detect_pkg/include/detect_pkg/detect.h`

## 当前 Tag 配置

代码中目前登记了 3 个 tag：

| ID | 用途 | `T_target_tag` 平移 (m) |
|---|---|---|
| 10 | box | `[-0.1475, 0.1255, 0.0305]` |
| 24 | 货架 | `[0.0, 0.0, 0.0]` |
| 36 | box（新增） | `[0.1275, 0.0875, 0.0625]` |

三个 tag 当前配置了相同旋转矩阵：

```text
 0  0 -1
 0  1  0
 1  0  0
```

重要：当前程序会把 `box_tag_ids` 中所有可见 tag 都当成同一目标的定位依据并进行融合。注释却说明 10/36 属于 box、24 属于货架。下一步必须确认业务意图：

- 如果服务只返回 box 位姿，tag 24 不应与 10/36 一起融合。
- 如果 tag 24 用于定位货架，应拆分目标配置或单独输出货架位姿。
- 如果 `[0,0,0]` 是刻意把货架 tag 当作目标原点，需明确目标坐标系名称，避免继续用 `box` 命名造成误解。

## 已完成

- AprilTag 检测、单 tag 位姿估计和多 tag 候选融合已实现。
- 位姿链路使用 Eigen 4x4 齐次矩阵，`camera2base` 的语义是 `T_base_camera`。
- 保留 `ids/rvec/tvec` 数量检查、候选数量检查和 Eigen 对齐保护。
- 单 tag 不进入 SVD；多个 tag 使用 SVD 将平均旋转投影回 SO(3)。
- 无论位置是否越界，都会打印 base 坐标系下完整 `x/y/z`。
- 每次服务请求最多保存一张调试图；失败时保存最后一次有效图像。
- `debug_img` 中只保留最新 10 张 PNG，文件名以可读时间戳命名。
- 已定位并解决运行时段错误：ARM64 环境中 `drawDebugStatus()` 调用 `cv::putText()` 后崩溃。当前该函数有意保持空实现，边框、坐标轴和图片保存不受影响。

## 当前待验证

新增 tag 36 后尚需真机验证：

1. 单独看到 tag 10、tag 36 时，计算出的目标位姿是否一致。
2. 同时看到 tag 10 和 36 时，融合结果是否稳定。
3. 同时看到 tag 24 与 10/36 时，是否错误融合了货架与 box。
4. 每次请求是否只新增一张图片，目录是否始终不超过 10 张。
5. 最新源码构建后是否仍无段错误，返回的 base 坐标方向是否符合机器人定义。

当前位置允许范围：`x=[0,1] m`、`y=[-0.5,0.5] m`、`z=[0,0.3] m`。超出范围是业务校验失败，不是检测或变换崩溃。

## 下一步

```bash
cd /home/user/detect_box_ws
colcon build --packages-select detect_pkg --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
source install/setup.bash
ros2 pkg prefix detect_pkg
ros2 run detect_pkg detect_server_node
```

依次测试只显示 10、只显示 36、同时显示 10/36、以及 24 与 box tag 同时出现的情况。根据结果决定是否把货架 tag 24 从 box 配置中拆出。

## 不要再踩的坑

- 不要混用 `/home/user/project/detect_box_ws`；实际运行工作区是 `/home/user/detect_box_ws`。
- 不要恢复 `cv::putText()`，除非先用最小程序或 GDB 确认 ARM64/OpenCV 问题已解决。
- 不要把 tag 24 和 box tag 融合，除非已经明确它们共享同一个目标坐标系。
- 不要改变 `camera2base` 的语义；它必须保持为 `T_base_camera`。
- 构建后必须重新 `source install/setup.bash`，并用 `ros2 pkg prefix detect_pkg` 核对实际运行来源。

