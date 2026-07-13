# Detect Box Workspace Handoff

## 工作区与任务

实际工作区是 `/home/user/detect_box_ws`，不是 `/home/user/project/detect_box_ws`。ROS 2 Humble，目标机器为 ARM64，主要节点是：

- 服务端源码：`src/detect_pkg/src/detect_sever_node.cpp`
- 服务端头文件：`src/detect_pkg/include/detect_pkg/detect.h`
- 可执行程序：`detect_pkg/detect_server_node`
- 服务名：`detect`
- 彩色图：`/camera/camera/color/image_raw`
- 彩色内参：`/camera/camera/color/camera_info`

任务是使用 RealSense 彩色图识别 AprilTag，根据配置好的 tag 在 box 上的安装位姿求出 box 在相机坐标系下的位姿，再通过固定外参 `T_base_camera` 转换成 box 在 base 坐标系下的位姿并通过 ROS 2 服务返回。同时保存调试图片、限制保存数量，并对结果位置做范围检查。

## 已完成

### 1. 段错误定位与处理

最初现象是打印完 tag 在相机坐标系下的正确位姿后立即 `Segmentation fault`。曾依次怀疑并替换 SVD、Eigen 4x4 运算、四元数转换，但增加阶段日志后确认以下步骤全部成功：

1. tag 到 box 候选位姿计算完成。
2. 候选写入 vector 完成。
3. 单 tag 候选融合完成。
4. `T_base_box` 计算完成。

最终确认崩溃发生在这些步骤之后，最接近的调用是 `drawDebugStatus()` 内的 `cv::putText()`。停用 `putText()` 后用户确认节点运行成功。

当前 `drawDebugStatus()` 是有意的空实现，只保留参数消除告警。AprilTag 边框、坐标轴和图片保存仍保留。不要在没有独立最小复现或 GDB/ASan 证据的情况下恢复 `cv::putText()`。

### 2. 位姿计算链路

用户要求恢复最初的 Eigen 齐次矩阵实现，当前链路为：

```text
T_camera_tag = OpenCV estimatePoseSingleMarkers 输出
T_camera_box = T_camera_tag * inverse(T_box_tag)
T_base_box   = T_base_camera * T_camera_box
```

其中：

```text
R_camera_box = R_camera_tag * transpose(R_box_tag)
t_camera_box = t_camera_tag - R_camera_box * t_box_tag
```

`buildPoseMessage()` 当前重新使用 `Eigen::Matrix4d` 构造 `box2camera`，然后执行：

```cpp
const Eigen::Matrix4d box2base = camera2base * box2camera;
```

再从 `box2base` 提取平移和旋转四元数。类中保留了 `EIGEN_MAKE_ALIGNED_OPERATOR_NEW`，并在构造四元数前把 3x3 block 求值为独立 `Eigen::Matrix3d`。

固定真机低腰外参目前是：

```text
 0.000000  -0.342020   0.939693   0.12972
-1.000000   0.000000   0.000000   0.03250
 0.000000  -0.939693  -0.342020   0.24561
 0.000000   0.000000   0.000000   1.00000
```

另外保留了以下健壮性处理：

- 检查 `ids`、`rotation_vectors`、`translation_vectors` 数量完全一致，防止越界。
- 检查旋转和平移候选数量一致且非空。
- 单 tag 时直接使用该候选，不进入不必要的 SVD。
- 多 tag 时仍使用 SVD 把平均旋转投影回 SO(3)。

### 3. XYZ 日志

只要 `detectBoxPose()` 成功返回，就会在位置范围检查前打印 box 在 base 坐标系下的完整坐标，即使随后判定越界也能看到：

```text
box在base坐标系下位置: x=..., y=..., z=... m
```

当前允许范围为：

- `x`: `[0.0, 1.0] m`
- `y`: `[-0.5, 0.5] m`
- `z`: `[0.0, 0.3] m`

测试中 tag 距相机约 2.2 m，因此转换后的 box 很可能超出上述范围。越界是业务校验失败，不是段错误。

### 4. 调试图片保存

此前一次请求最多尝试 5 帧，每次失败都立即保存，因此一次请求会生成 5 张图。当前已改为：

- 检测成功：保存成功帧一张。
- 所有尝试失败：循环中只缓存最后一张调试图，整个请求结束后保存一张。
- 等待图像阶段完全没有拿到调试图时，不保存空图。
- 每次保存后按文件修改时间清理，只保留 `debug_img` 中最新 10 张 PNG。

文件名以本地时间开头，例如：

```text
20260713_174523_123456789_success.png
20260713_174530_987654321_failed.png
```

清理逻辑现在会处理 `debug_img` 下所有 `.png`，不再只匹配旧的 `apriltag_detection_` 前缀。该目录应保持为本节点专用目录，否则可能删除其中其他 PNG。

## 当前未闭环

最后一次组合修改包括：恢复 Eigen 齐次矩阵链路、每请求最多保存一图、时间文件名、完整 XYZ 日志。`colcon build` 已报告成功，但用户尚未提供这版代码的真机完整运行日志。因此下一会话的第一件事不是继续重构，而是现场复测当前版本。

还应确认：

- 一次失败请求是否只新增一张图片。
- 连续请求后目录是否稳定保持最多 10 张 PNG。
- 成功和越界失败路径是否都不再崩溃。
- 返回的 base 坐标数值及坐标轴方向是否符合机器人实际定义。
- 实际运行的可执行程序确实来自 `/home/user/detect_box_ws/install`。

## 下一步计划

1. 在 `/home/user/detect_box_ws` 中重新构建并加载环境：

   ```bash
   colcon build --packages-select detect_pkg --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
   source /home/user/detect_box_ws/install/setup.bash
   ros2 pkg prefix detect_pkg
   ```

2. 启动相机和 `detect_server_node`，请求一次服务，保存完整服务端日志。
3. 检查日志是否依次经过候选计算、融合、base 变换、XYZ 打印和图片保存。
4. 检查 `debug_img` 的新增文件数、命名和总数量。
5. 若仍出现段错误，不再靠最后一条日志猜测。使用带符号的 `RelWithDebInfo` 或 Debug 构建，在 GDB 中运行并获取 `bt full`；必要时单独构建 ASan 版本。
6. 位姿链路稳定后，再根据机器人真实工作空间决定是否调整 `validateBoxPosePosition()` 的范围。
7. 若必须恢复图片文字叠加，先写一个只对同类型 `cv::Mat` 调用 `cv::putText()` 的最小 ARM64 测试，确认 OpenCV ABI、图像步长和图像类型，再恢复到节点。

## 绝对不要再踩的坑

1. **不要混淆工作区。** 早期 IDE 指向 `/home/user/project/detect_box_ws`，终端运行 `/home/user/detect_box_ws`，日志和源码因此对不上。每次先执行 `pwd`、`ros2 pkg prefix detect_pkg`，并核对源码、build、install 的路径。
2. **不要把“最后一条日志”当作崩溃行。** 日志之后可能还有字符串、绘图、保存等操作。先加阶段日志或直接取 GDB 栈。
3. **不要再把外参数值当成段错误根因。** 错误矩阵会导致错误位姿，通常不会导致 SIGSEGV。本次坐标变换已经通过阶段日志。
4. **不要恢复 `cv::putText()` 后直接上真机。** 它是目前唯一通过排除法与运行结果锁定的崩溃触发点。
5. **不要在每次检测尝试中保存失败图片。** `max_detection_attempts=5` 会导致一次请求保存 5 张。失败图必须在请求结束后只保存最后一张。
6. **不要假设 OpenCV 输出 vector 永远等长。** 保留 `ids/rvec/tvec` 数量检查，避免潜在越界直接变成 SIGSEGV。
7. **不要删除 Eigen 对齐和显式求值保护。** 类含固定尺寸 Eigen 成员，ARM64 上应谨慎处理对齐；四元数构造使用独立 3x3 矩阵。
8. **不要在未确认坐标系语义时调外参。** `camera2base` 在代码中的语义必须始终是 `T_base_camera`，即把相机坐标表达的点转换到 base 坐标表达。
9. **不要把 RealSense 的 `depth_to_color` 外参用于当前彩色 AprilTag 链路。** `/camera/camera/extrinsics/depth_to_color` 是深度光学坐标系到彩色光学坐标系的固定外参；当前节点只用彩色图和对应内参，不需要该话题。
10. **不要忘记重新 source。** 构建后必须 `source /home/user/detect_box_ws/install/setup.bash`，否则 `ros2 run` 可能启动其他 overlay 中的旧程序。

