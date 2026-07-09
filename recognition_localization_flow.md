# 识别与定位流程模式说明

本文说明当前项目中相机图像、AprilTag 识别、box 位姿定位与节点通信之间的关系，只描述流程模式和接口边界，不展开具体启动命令。

## 总体模式

项目采用“相机发布”和“检测服务”分离的模式。

相机侧节点只负责采集或生成图像数据，并发布 ROS topic。检测侧节点 `detect_server_node` 不启动相机、不直接管理 RealSense SDK，也不主动拉取相机数据；它只订阅图像与相机内参，在本地缓存最新帧。当上层流程需要视觉定位结果时，通过 ROS2 service 向检测节点发起一次请求，检测节点再基于缓存图像执行识别与定位。

这种模式把相机生命周期和检测服务生命周期解耦：相机可以来自真实 D435、仿真 D435，或其他兼容 ROS 图像接口的发布者；检测节点只关心 topic 数据格式和相机内参是否有效。

## 节点职责

### 相机发布节点

相机发布节点负责产生视觉输入。项目中 `d435_publisher` 包封装了 D435i 相机相关启动逻辑，包含彩色图、深度图、相机内参以及可选的相机外参 TF 发布。对检测服务而言，最关键的数据是：

- `sensor_msgs/msg/Image`：彩色图像。
- `sensor_msgs/msg/CameraInfo`：彩色相机内参。

检测服务默认只使用彩色图像和相机内参，不使用深度图完成 AprilTag 位姿估计。

### 检测服务节点

`detect_pkg` 中的 `detect_server_node` 是识别与定位的核心节点。它的职责是：

- 订阅彩色图像 topic，并缓存最新一帧图像。
- 订阅相机内参 topic，并缓存最新相机参数。
- 提供 `upper_limb_interface/srv/DetectAprilTag` 服务。
- 在收到服务请求后等待新图像，执行 AprilTag 检测和位姿估计。
- 将 tag 位姿换算为 box 位姿，再转换到 `base_link` 坐标系。
- 返回成功标志、诊断信息和 box 位姿。
- 在检测与定位成功时保存本次识别画面到 `debug_img`。

该节点内部维护 `camera2base` 外参矩阵。也就是说，虽然相机包可以发布 TF，但当前检测服务的 box 位姿转换并不是运行时查 TF，而是使用源码中的外参矩阵完成 `camera` 到 `base_link` 的变换。

### 上层调用节点

上层调用方可以是测试客户端、抓取流程节点或其他任务节点。调用方不直接读取相机图像，也不参与 AprilTag 识别细节；它只通过检测服务请求一次结果，并根据响应中的 `success`、`message` 和 `box_pose` 决定后续行为。

## 数据流

整体数据流可以概括为：

```text
相机发布节点
  -> 彩色图像 topic
  -> 相机内参 topic
  -> detect_server_node 缓存最新图像和内参
  -> 上层节点发起 DetectAprilTag 服务请求
  -> detect_server_node 执行 AprilTag 识别与定位
  -> 返回 box_pose in base_link
```

检测节点在服务请求到来之前持续接收图像，但不会每一帧都执行定位。定位计算由服务请求触发，这使视觉检测变成按需计算模式，而不是固定频率发布模式。

## 识别流程

服务请求触发后，检测节点按如下步骤处理：

1. 等待一帧比上次服务请求更新的彩色图像，同时确认相机内参有效。
2. 将彩色图像转换为灰度图。
3. 使用 OpenCV ArUco/AprilTag 接口检测 `DICT_APRILTAG_36h11` 标签。
4. 对识别出的 tag 角点执行单 tag 位姿估计，得到每个 tag 在相机坐标系下的位姿 `T_cam_tag`。
5. 根据参数中登记的 `box_tag_ids` 过滤 tag，只保留属于目标 box 的 tag。
6. 对每个匹配 tag，结合该 tag 在 box 坐标系中的安装位姿 `T_box_tag`，反推出 box 在相机坐标系下的候选位姿 `T_cam_box`。
7. 如果多个目标 tag 同时可见，对多个候选位姿进行融合。
8. 使用内部 `camera2base` 外参矩阵，将 `T_cam_box` 转换为 `T_base_box`。
9. 对转换后的 box 位置做范围校验。
10. 成功时返回定位结果；失败时返回 `fallback_pose` 指定的位姿。

其中核心坐标关系是：

```text
T_cam_tag = T_cam_box * T_box_tag
T_cam_box = T_cam_tag * inverse(T_box_tag)
T_base_box = T_base_camera * T_cam_box
```

源码中变量命名使用 `camera2base` 表示从相机坐标系到 `base_link` 坐标系的外参矩阵。

## 多 Tag 融合模式

box 上可以配置多个 AprilTag。每个 tag 都有独立的 ID、相对 box 原点的位置和相对 box 坐标系的旋转矩阵。

当画面中只识别到一个目标 tag 时，检测节点直接使用该 tag 推导出的 box 位姿。当识别到多个目标 tag 时，节点会分别计算每个 tag 对应的 `T_cam_box` 候选值，然后融合：

- 平移部分采用算术平均。
- 旋转部分先对旋转矩阵求平均，再通过 SVD 投影回合法旋转矩阵。

这种融合方式可以降低单个 tag 检测噪声对最终 box 位姿的影响，但它依赖 tag 尺寸、tag 安装位置和相机内参的一致性。

## 失败返回模式

检测服务响应中始终包含 `box_pose`。当检测成功时，`box_pose` 是估计出的 box 位姿。当未识别到目标 tag、tag 位姿估计失败、位置校验失败或等待图像失败时，`success` 为 `false`，`box_pose` 返回 `fallback_pose` 参数指定的位姿。

`fallback_pose` 的格式是：

```text
[x, y, z, qx, qy, qz, qw]
```

该参数用于给上层流程一个明确、可配置的失败位姿，而不是固定返回单位位姿。节点会校验该数组长度和数值合法性，并对四元数做归一化。

## 接口通信模式

### Topic 输入

检测节点作为 topic 订阅者接收相机数据：

```text
sensor_msgs/msg/Image
sensor_msgs/msg/CameraInfo
```

图像回调只做轻量的数据转换和缓存，不直接执行完整识别。这样可以避免检测计算阻塞相机订阅回调。

### Service 请求响应

检测节点提供服务：

```text
upper_limb_interface/srv/DetectAprilTag
```

服务请求字段：

```text
bool capture_once
```

当前实现中，该字段主要表示“触发一次检测”的语义；服务端收到请求后即尝试执行一次检测流程。

服务响应字段：

```text
bool success
string message
geometry_msgs/Pose box_pose
```

响应语义如下：

- `success` 表示本次是否成功估计到有效 box 位姿。
- `message` 描述检测结果、使用的 tag 数量或失败原因。
- `box_pose` 在成功时表示 box 坐标系在 `base_link` 下的位姿，失败时表示 `fallback_pose`。

### 参数接口

检测节点通过 ROS2 参数配置检测行为和几何标定数据。关键参数包括：

- `image_topic` 与 `camera_info_topic`：指定输入图像和内参来源。
- `box_tag_ids`：指定哪些 tag 属于目标 box。
- `box_tag_positions_m`：指定每个 tag 原点在 box 坐标系下的位置。
- `box_tag_rotations_row_major`：指定每个 tag 坐标系相对 box 坐标系的旋转。
- `tag_size_m`：指定 AprilTag 黑色方框边长。
- `fallback_pose`：指定失败时返回的位姿。
- `debug_image_save_dir`：指定成功帧和调试图像保存目录。

其中 `fallback_pose` 支持运行时动态更新；检测失败后的下一次服务响应会使用更新后的位姿。

## 调试图像模式

检测节点在成功完成识别与定位时，会保存本次检测图像到项目根目录下的 `debug_img`。图像文件名带时间戳，避免覆盖历史结果。

当调试图像开关打开时，节点还会在图像中绘制 tag 边框、tag ID、坐标轴和状态文字，并可保存失败帧用于排查识别问题。

## 边界与假设

当前检测服务有几个重要边界：

- 检测节点只消费 ROS 图像和内参，不负责相机启动。
- 检测节点当前使用内部 `camera2base` 矩阵完成坐标转换，不在服务回调中查询 TF。
- AprilTag 尺寸、tag 安装位姿、相机内参和相机外参必须彼此一致，否则返回的 box 位姿会系统性偏移。
- 服务是按需触发模式，不是持续发布定位结果的 topic 模式。
- 上层节点应以 `success` 作为判断本次定位是否有效的主信号，不应仅凭 `box_pose` 是否非零判断成功。
