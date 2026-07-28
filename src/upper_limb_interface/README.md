# upper_limb_interface

`upper_limb_interface` 是上肢操作相关的 ROS2 接口包，只负责定义并生成 `srv`、`action` 等通信接口，不包含业务节点。

该包应被其他功能包依赖，例如检测服务包、上肢执行节点、导航与上肢协作节点等。

## 目录结构

```text
upper_limb_interface/
├── CMakeLists.txt
├── package.xml
├── action/
│   └── UpperLimbManipulate.action
└── srv/
    └── DetectAprilTag.srv
```

## 接口说明

### DetectAprilTag.srv

历史命名保持不变，用于触发一次箱体检测，并返回 box 当前观察位姿。

```srv
bool capture_once
---
bool success
string message
geometry_msgs/Pose box_pose
```

字段说明：

- `capture_once`：触发一次检测。当前检测节点收到请求即采集一个短时间深度窗口执行检测。
- `success`：是否成功估计到当前观察位姿。
- `message`：检测结果说明或失败原因。
- `box_pose`：box 坐标系在 `base_link` 坐标系下的位姿。失败时由服务端返回单位位姿。

### UpperLimbManipulate.action

用于导航工程与上肢操作之间的任务级协作。

```action
string command
string waypoint_id
---
bool success
string message
---
```

字段说明：

- `command`：操作命令，例如 `pick`、`place`。
- `waypoint_id`：导航侧提供的航点或任务点 ID，例如 `pickup_a`、`drop_b`。
- `success`：上肢任务是否执行成功。
- `message`：执行结果说明或错误原因。

## 编译

在工作空间根目录执行：

```bash
colcon build --packages-select upper_limb_interface
source install/setup.bash
```

如其他包依赖本接口包，建议一起编译：

```bash
colcon build --packages-select upper_limb_interface detect_pkg
```

## 使用约定

- 接口字段一旦被其他包使用，修改前需要同步通知相关开发者。
- 新增接口时优先放在本包内，避免业务包之间直接复制 `.srv` 或 `.action`。
- 修改接口后必须重新编译依赖该接口的包，并重新 `source install/setup.bash`。
- Foxglove、ros2 service、ros2 action 无法识别接口时，优先检查是否已编译并 source 了本包。

## 依赖

主要依赖：

- `rosidl_default_generators`
- `rosidl_default_runtime`
- `geometry_msgs`
- `std_msgs`

本包是接口包，`package.xml` 中已声明：

```xml
<member_of_group>rosidl_interface_packages</member_of_group>
```
