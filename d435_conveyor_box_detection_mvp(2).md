# D435 传送带开口箱检测与观察定位 MVP（C++17）

## 1. 文档目的

本文档用于指导 Codex 逐步实现一套基于 Intel RealSense D435 的传送带开口箱检测系统。

本阶段只实现以下目标：

1. 主控约每 3 秒发起一次检测请求；
2. 每次请求内部连续采集 0.6～1.0 秒深度图；
3. 不依赖长期固定背景，检测自右向左运动的箱体；
4. 在相机坐标系中完成检测与当前观察位姿估计；
5. 最后通过相机到机器人 `base` 的外参变换得到 `base` 坐标；
6. 传送带运动期间检测出的位姿只用于观察、显示、记录和提示；
7. 本阶段不输出可供机器人直接抓取的最终位姿。

核心原则：

```text
检测到箱体 ≠ 可以抓取
检测到当前位姿 ≠ 最终抓取位姿有效
```

第一阶段统一要求：

```cpp
result.pose_valid_for_grasp = false;
```

---

## 2. 场景与约束

### 2.1 已知条件

- 相机：Intel RealSense D435；
- 相机安装在机器人胸部；
- 机器人下肢固定；
- 机器人每次站到传送带前的位置会有小幅出入；
- 因此相机画面不能与历史画面保证逐像素对齐；
- 传送带由人工目测后手动启停；
- 滚道运动方向为自右向左；
- 主控约每 3 秒调用一次视觉逻辑；
- 无法从 PLC 或编码器获得传送带速度；
- 相机视野较小；
- 箱体尚未到达理想位置时，视野内可能只有箱体左侧一小部分；
- 箱体和滚道颜色接近，均为绿色；
- 箱体为开口长方体；
- 箱体外部长、宽、高和凹槽尺寸可预先测量。

### 2.2 本阶段不实现

- 动态抓取；
- 预测人工停止时刻；
- 预测人工停止后的最终箱体位置；
- 自动控制传送带；
- 判断人工停止位置是否理想；
- 停止后的高精度最终抓取定位；
- 机器人抓取轨迹规划；
- 完整 CAD 模型 ICP 配准。

---

## 3. 最新总体方案

本阶段采用：

> 方案B：短时间连续深度帧的间隔帧差分生成运动候选，再回到当前完整深度图，利用三维点云和开口箱几何完成确认与观察定位。

不再把“长期固定的空传送带深度背景”作为第一版主方案。

原因：

1. 机器人每次站位略有不同；
2. 栏杆、滚道和支架在图像中的位置会变化；
3. 固定背景会在深度突变边缘产生大量假前景；
4. 当前阶段主要处理运动中的箱体；
5. 同一次短时间采样窗口内，栏杆和支架相对于相机基本静止；
6. 运动箱体会在连续深度帧中产生稳定变化。

总体链路：

```text
主控发起 DetectOnce
        ↓
连续采集 0.8 秒左右深度帧
        ↓
间隔帧深度差分
        ↓
多组差分累计投票
        ↓
提取运动候选
        ↓
按“自右向左运动”的先验扩大候选框
        ↓
从当前完整深度图提取三维点云
        ↓
在相机坐标系中完成三维过滤
        ↓
估计箱体上沿近似平面
        ↓
将三维上沿点投影到局部米制二维平面
        ↓
二维 RANSAC 拟合长边和短边
        ↓
验证内部凹槽
        ↓
根据当前可见边数量选择定位策略
        ↓
输出相机坐标系观察位姿
        ↓
通过 T_base_camera 转换到 base
        ↓
输出运动状态和几何状态
        ↓
pose_valid_for_grasp = false
```

---

## 4. 坐标系设计

### 4.1 不建立传送带坐标系

本方案不要求定义或维护专门的传送带坐标系。

箱体检测、点云处理、平面拟合、边线拟合和观察位姿估计均在相机坐标系中完成。

主要坐标系：

```text
camera_depth_optical_frame
base_link
```

### 4.2 D435 光学坐标系

通常约定：

```text
Xcamera：图像右方
Ycamera：图像下方
Zcamera：相机正前方
```

相机安装在胸部并向下倾斜，因此：

- 不能直接把 `Xcamera-Ycamera` 平面当作俯视平面；
- 不能简单把某个相机轴当作滚道高度轴；
- 应先估计箱体上沿的三维平面，再在该平面中构造局部二维基底。

### 4.3 相机到 base 的外参

箱体相对于相机的位姿：

```text
T_camera_box
```

相机相对于机器人 base 的外参：

```text
T_base_camera
```

最终：

```text
T_base_box = T_base_camera * T_camera_box
```

点坐标转换：

```cpp
Eigen::Vector3f point_base =
    T_base_camera * point_camera;
```

如果机器人胸部、腰部和相机安装姿态固定，`T_base_camera` 可使用固定标定值。

如果腰部或胸部关节角会变化，则必须由机器人正运动学实时计算 `T_base_camera`。

机器人整体站位相对于传送带发生小幅变化，不会改变相机相对于机器人自身 `base` 的外参。

---

## 5. 技术栈

推荐：

```text
C++17
librealsense2
OpenCV
Eigen3
yaml-cpp
CMake
```

不使用 GoogleTest。

测试方式：

1. 普通 C++ 测试可执行程序；
2. `assert`；
3. D435 深度序列录制与离线回放；
4. 保存调试图、JSON 和 CSV；
5. 使用真实现场数据重复验证。

第一阶段不强制依赖 PCL，三维点可使用：

```cpp
std::vector<Eigen::Vector3f>
```

---

## 6. 推荐目录结构

```text
box_vision/
├── CMakeLists.txt
├── config/
│   └── box_vision.yaml
├── include/box_vision/
│   ├── types.hpp
│   ├── frame_provider.hpp
│   ├── motion_mask_builder.hpp
│   ├── candidate_extractor.hpp
│   ├── point_cloud_builder.hpp
│   ├── plane_ransac_3d.hpp
│   ├── line_ransac_2d.hpp
│   ├── box_geometry_estimator.hpp
│   ├── motion_estimator.hpp
│   ├── detection_service.hpp
│   ├── debug_visualizer.hpp
│   └── data_recorder.hpp
├── src/
│   ├── frame_provider.cpp
│   ├── motion_mask_builder.cpp
│   ├── candidate_extractor.cpp
│   ├── point_cloud_builder.cpp
│   ├── plane_ransac_3d.cpp
│   ├── line_ransac_2d.cpp
│   ├── box_geometry_estimator.cpp
│   ├── motion_estimator.cpp
│   ├── detection_service.cpp
│   ├── debug_visualizer.cpp
│   ├── data_recorder.cpp
│   └── main.cpp
├── tools/
│   ├── record_depth_sequence.cpp
│   ├── replay_depth_sequence.cpp
│   ├── test_motion_estimator.cpp
│   ├── test_plane_ransac.cpp
│   ├── test_line_ransac.cpp
│   └── test_geometry_estimator.cpp
└── data/
    ├── recorded_sequences/
    └── debug_output/
```

---

## 7. 箱体尺寸模型

```cpp
struct BoxModel
{
    float outer_length_m = 0.0F;
    float outer_width_m = 0.0F;
    float outer_height_m = 0.0F;

    float rim_thickness_m = 0.0F;

    float inner_length_m = 0.0F;
    float inner_width_m = 0.0F;
    float cavity_depth_m = 0.0F;
};
```

配置示例：

```yaml
box_model:
  outer_length_m: 0.600
  outer_width_m: 0.400
  outer_height_m: 0.280
  rim_thickness_m: 0.025
  inner_length_m: 0.550
  inner_width_m: 0.350
  cavity_depth_m: 0.220
```

示例值必须替换为现场实测值。

---

## 8. 状态设计

### 8.1 运动状态

```cpp
enum class MotionState : std::uint8_t
{
    UNKNOWN = 0,
    MOVING = 1,
    STATIC_CANDIDATE = 2
};
```

`STATIC_CANDIDATE` 只表示短时间位置变化较小，不代表完成静态精定位。

### 8.2 几何状态

```cpp
enum class GeometryState : std::uint8_t
{
    NO_BOX = 0,

    // 只确认存在自右向左运动的目标，
    // 几何信息不足。
    PARTIAL_ENTERING = 1,

    // 检测到部分箱体边线，
    // 但不足以恢复完整中心。
    PARTIAL_GEOMETRY = 2,

    // 根据可见短边、长边和已知尺寸
    // 推算出观察中心。
    ESTIMATED_CENTER = 3,

    // 当前帧中长短边、凹槽信息较充分。
    SUFFICIENT_GEOMETRY = 4,

    GEOMETRY_FAILED = 5
};
```

### 8.3 综合状态

```cpp
enum class DetectionStatus : std::uint8_t
{
    NO_BOX = 0,
    MOVING_PARTIAL_ENTERING = 1,
    MOVING_PARTIAL_GEOMETRY = 2,
    MOVING_ESTIMATED_CENTER = 3,
    MOVING_SUFFICIENT_GEOMETRY = 4,
    MOTION_UNCERTAIN = 5,
    STATIC_CANDIDATE = 6,
    DETECTION_FAILED = 7
};
```

---

## 9. 核心数据结构

```cpp
struct DepthFrame
{
    cv::Mat depth_u16;
    double timestamp_sec = 0.0;
    rs2_intrinsics intrinsics{};
    float depth_scale_m = 0.001F;
    int sequence_index = -1;
};

struct Pose3D
{
    Eigen::Vector3f position_camera =
        Eigen::Vector3f::Zero();

    Eigen::Vector3f position_base =
        Eigen::Vector3f::Zero();

    float roll_rad = 0.0F;
    float pitch_rad = 0.0F;
    float yaw_rad = 0.0F;
};

struct VisibilityEstimate
{
    bool touches_left_border = false;
    bool touches_right_border = false;
    bool touches_top_border = false;
    bool touches_bottom_border = false;

    float visible_length_m = 0.0F;
    float visible_width_m = 0.0F;

    float length_visible_ratio = 0.0F;
    float width_visible_ratio = 0.0F;
};

struct GeometryEstimate
{
    bool valid = false;

    GeometryState geometry_state =
        GeometryState::GEOMETRY_FAILED;

    Pose3D observation_pose;
    VisibilityEstimate visibility;

    bool front_short_edge_visible = false;
    bool back_short_edge_visible = false;
    bool long_edge_a_visible = false;
    bool long_edge_b_visible = false;
    bool cavity_verified = false;

    float cavity_score = 0.0F;
    float geometry_confidence = 0.0F;
    float line_fit_rmse_m = 0.0F;
    float plane_fit_rmse_m = 0.0F;

    float motion_reference_m = 0.0F;
    bool motion_reference_valid = false;
};

struct BoxObservation
{
    bool valid = false;
    double timestamp_sec = 0.0;
    GeometryEstimate geometry;
    cv::Rect image_bbox;
    float detection_confidence = 0.0F;
};

struct DetectionResult
{
    DetectionStatus status =
        DetectionStatus::DETECTION_FAILED;

    MotionState motion_state =
        MotionState::UNKNOWN;

    GeometryState geometry_state =
        GeometryState::GEOMETRY_FAILED;

    bool box_detected = false;
    bool pose_detected = false;

    // MVP 阶段必须始终为 false。
    bool pose_valid_for_grasp = false;

    Pose3D observation_pose;

    float estimated_velocity_mps = 0.0F;
    float position_span_m = 0.0F;
    float velocity_fit_rmse_m = 0.0F;

    float motion_confidence = 0.0F;
    float detection_confidence = 0.0F;
    float geometry_confidence = 0.0F;

    int total_frame_count = 0;
    int valid_frame_count = 0;

    double latest_sensor_timestamp_sec = 0.0;

    std::string message;
};
```

---

## 10. D435 连续采集

D435 建议常驻运行，不要每 3 秒重新启动相机。

每次请求只收集一个短窗口：

```text
推荐窗口：0.8 秒
可调范围：0.6～1.0 秒
推荐帧率：30 FPS
理论帧数：约 24 帧
```

接口：

```cpp
class D435FrameProvider
{
public:
    D435FrameProvider(
        int width,
        int height,
        int fps);

    bool start();
    void stop();

    bool getDepthFrame(
        DepthFrame& output,
        int timeout_ms);

private:
    rs2::pipeline pipeline_;
    rs2::config config_;
    rs2::pipeline_profile profile_;

    float depth_scale_m_ = 0.001F;
    int sequence_index_ = 0;
};
```

注意：

- `cv::Mat` 必须 `clone()`；
- 使用相机帧真实时间戳；
- 不使用主控 3 秒周期作为帧间 `dt`；
- 检查时间戳单调递增；
- 深度无效值为 0。

---

## 11. 方案B：间隔帧深度差分

### 11.1 单组差分

设两帧：

```text
D_old
D_new
```

只比较两帧都有效的像素：

```text
D_old > 0
D_new > 0
```

运动条件：

```text
abs(D_new - D_old) > threshold
```

推荐阈值起点：

```text
20～35 mm
```

C++：

```cpp
cv::Mat buildPairMotionMask(
    const cv::Mat& depth_old_u16,
    const cv::Mat& depth_new_u16,
    float depth_scale_m,
    float threshold_m)
{
    const int threshold_raw =
        static_cast<int>(
            threshold_m / depth_scale_m);

    cv::Mat valid =
        (depth_old_u16 > 0) &
        (depth_new_u16 > 0);

    cv::Mat abs_diff;
    cv::absdiff(
        depth_old_u16,
        depth_new_u16,
        abs_diff);

    return valid &
        (abs_diff > threshold_raw);
}
```

### 11.2 间隔帧而非相邻帧

建议：

```text
frame_gap = 4～8
```

例如 30 FPS、`frame_gap = 5`，间隔约 0.167 秒。

### 11.3 多组差分投票

计算：

```text
D0 与 D5
D1 与 D6
D2 与 D7
...
```

累计变化票数：

```cpp
cv::Mat vote_count =
    cv::Mat::zeros(
        image_size,
        CV_16UC1);
```

最后保留：

```text
vote_count >= min_motion_votes
```

推荐初值：

```yaml
motion_mask:
  frame_gap: 5
  min_motion_votes: 3
  depth_difference_threshold_m: 0.025
```

随后执行：

- 闭运算；
- 开运算；
- 可选膨胀。

重要：

> 运动掩膜只用于找到候选位置，不直接用于箱体完整几何定位。

---

## 12. 运动候选与向右扩张

滚道运动方向：

```text
右 → 左
```

箱体最先进入视野的是左侧前沿，而箱体主体位于该前沿右侧。

帧间差分通常只得到一条较窄的变化带，因此候选框需要主要向右扩张。

```cpp
cv::Rect expandCandidateRightward(
    const cv::Rect& motion_bbox,
    const cv::Size& image_size,
    int expand_left_px,
    int expand_right_px,
    int expand_up_px,
    int expand_down_px);
```

初始建议：

```yaml
candidate_expand:
  left_px: 20
  right_px: 250
  up_px: 50
  down_px: 80
```

扩大候选框只能包含已经进入视野的箱体部分，无法恢复仍处于视野外的数据。

---

## 13. 箱体左侧只进入一部分时的影响

### 13.1 对运动检测影响较小

左侧前沿会在连续帧中稳定向左移动，方案B通常能够发现该运动区域。

### 13.2 对完整定位影响较大

禁止使用：

```cpp
center = mean(visible_points);
```

因为随着箱体不断进入视野，可见点云中心会同时受到：

```text
真实运动
+
视野裁剪变化
```

的影响。

### 13.3 视野边界判断

```cpp
bool touches_right_border =
    bbox.x + bbox.width >=
    image_width - border_margin_px;
```

由于箱体从右侧进入：

```text
touches_right_border = true
```

通常意味着箱体尚未完全进入视野。

应优先输出：

```text
PARTIAL_ENTERING
```

而不是误判为一个尺寸较小的完整箱体。

---

## 14. 从完整深度图提取点云

运动掩膜只用于生成候选框。

几何定位时，应回到当前完整深度图提取候选框中的点云。

可选择：

- 短窗口最后一帧；
- 最后一个高质量帧；
- 候选可见范围最大的一帧。

接口：

```cpp
struct PointSample
{
    Eigen::Vector3f point_camera =
        Eigen::Vector3f::Zero();

    cv::Point pixel;
};

class PointCloudBuilder
{
public:
    std::vector<PointSample> build(
        const DepthFrame& frame,
        const cv::Rect& candidate_bbox) const;
};
```

使用：

```cpp
rs2_deproject_pixel_to_point
```

将深度像素反投影到相机坐标系。

---

## 15. 相机坐标系三维 ROI

即使不建立传送带坐标系，也要使用相机坐标系三维 ROI。

```yaml
camera_roi_3d:
  x_min_m: -0.80
  x_max_m: 0.80
  y_min_m: -0.50
  y_max_m: 1.20
  z_min_m: 0.30
  z_max_m: 2.00
```

实际数值必须根据现场安装调整。

作用：

- 排除机器人身体；
- 排除过近和过远物体；
- 排除图像中明显无关区域；
- 降低栏杆和支架干扰；
- 降低点云数量。

---

## 16. RANSAC 的最终使用方式

采用混合方式：

```text
三维点云
    ↓
三维 RANSAC 或受约束平面估计
    ↓
得到箱体上沿近似平面
    ↓
三维上沿点投影到局部米制二维平面
    ↓
二维 RANSAC 拟合长边和短边
```

二维点不是图像像素，而是三维点在上沿平面内的米制坐标。

这样同时保留：

- 三维尺度；
- 相机坐标；
- 平面内真实长宽关系；
- 更简单的平行、垂直和角点计算。

---

## 17. 三维平面 RANSAC

平面形式：

```text
n · p + d = 0
```

结构：

```cpp
struct Plane3D
{
    Eigen::Vector3f normal =
        Eigen::Vector3f::Zero();

    float d = 0.0F;

    Eigen::Vector3f centroid =
        Eigen::Vector3f::Zero();

    std::vector<std::size_t>
        inlier_indices;

    float rmse_m = 0.0F;
};
```

注意不能直接对整个候选点云拟合最大平面，否则可能拟合到：

- 箱体侧壁；
- 栏杆；
- 内腔底部；
- 箱内物品表面。

因此应先构造“上沿候选点”，可组合：

1. 候选框较上区域；
2. 深度梯度边缘；
3. 靠近相机的一层点；
4. 局部平面性；
5. 已知箱体尺寸约束。

第一版可先采用：

```text
图像上部
+
深度梯度
+
靠近相机深度分位层
```

作为上沿候选，再做平面 RANSAC。

---

## 18. 局部平面二维基底

```cpp
struct PlaneBasis
{
    Eigen::Vector3f origin_camera;
    Eigen::Vector3f axis_u_camera;
    Eigen::Vector3f axis_v_camera;
    Eigen::Vector3f normal_camera;
};
```

构造：

```cpp
PlaneBasis makePlaneBasis(
    const Plane3D& plane)
{
    PlaneBasis basis;

    basis.origin_camera =
        plane.centroid;

    basis.normal_camera =
        plane.normal.normalized();

    Eigen::Vector3f reference =
        Eigen::Vector3f::UnitX();

    if (std::abs(
            basis.normal_camera.dot(
                reference)) > 0.90F)
    {
        reference =
            Eigen::Vector3f::UnitY();
    }

    basis.axis_u_camera =
        basis.normal_camera
            .cross(reference)
            .normalized();

    basis.axis_v_camera =
        basis.normal_camera
            .cross(
                basis.axis_u_camera)
            .normalized();

    return basis;
}
```

三维投影到二维：

```cpp
Eigen::Vector2f projectToPlane(
    const Eigen::Vector3f& point_camera,
    const PlaneBasis& basis)
{
    const Eigen::Vector3f delta =
        point_camera -
        basis.origin_camera;

    return Eigen::Vector2f(
        delta.dot(
            basis.axis_u_camera),
        delta.dot(
            basis.axis_v_camera));
}
```

二维还原到三维：

```cpp
Eigen::Vector3f unprojectFromPlane(
    const Eigen::Vector2f& uv,
    const PlaneBasis& basis)
{
    return
        basis.origin_camera +
        uv.x() *
            basis.axis_u_camera +
        uv.y() *
            basis.axis_v_camera;
}
```

---

## 19. 二维直线 RANSAC

结构：

```cpp
struct Line2D
{
    Eigen::Vector2f point =
        Eigen::Vector2f::Zero();

    Eigen::Vector2f direction =
        Eigen::Vector2f::Zero();

    Eigen::Vector2f normal =
        Eigen::Vector2f::Zero();

    std::vector<std::size_t>
        inlier_indices;

    float visible_length_m = 0.0F;
    float rmse_m = 0.0F;
};
```

流程：

1. 随机选两个二维点；
2. 生成候选直线；
3. 统计点到直线距离小于阈值的内点；
4. 重复 100～300 次；
5. 选择内点最多、跨度合理的模型；
6. 使用全部内点 PCA 精拟合；
7. 删除内点；
8. 继续拟合下一条边。

推荐：

```yaml
line_ransac:
  max_iterations: 200
  inlier_distance_m: 0.012
  min_inliers: 25
  min_visible_length_m: 0.100
  parallel_tolerance_deg: 8.0
  perpendicular_tolerance_deg: 10.0
```

---

## 20. 凹槽验证

凹槽用于区分：

- 目标开口箱；
- 平板；
- 栏杆；
- 普通实心长方体；
- 其他运动物体。

采用两级验证。

### 初步候选

满足：

- 有稳定运动区域；
- 深度和尺寸大致合理；
- 有部分上沿或侧壁；
- 自右向左运动。

可输出：

```text
PARTIAL_ENTERING
```

不强制完整凹槽验证。

### 确认开口箱

满足：

- 至少一条可靠上沿边；
- 边线一侧存在低于上沿平面的内部点；
- 内部点数量和高度差达到阈值。

凹槽分数：

```text
cavity_score =
    low_inner_point_count /
    valid_inner_point_count
```

推荐：

```yaml
cavity:
  min_cavity_drop_m: 0.040
  min_inner_points_partial: 20
  min_inner_points_confirmed: 80
  min_cavity_score_partial: 0.20
  min_cavity_score_confirmed: 0.35
```

---

## 21. 判断箱体内部方向

只看到一条边时，需要判断箱体内部位于边线哪一侧。

方法：

1. 在线两侧建立窄条区域；
2. 统计两侧低于上沿平面的点；
3. 比较点数、平均下凹深度和尺寸合理性；
4. 凹槽点更多的一侧视为箱体内部。

输出：

```cpp
Eigen::Vector2f inward_normal;
```

该方向用于：

- 推算中心；
- 区分前沿和后沿；
- 验证凹槽；
- 判断长边、短边的内侧。

---

## 22. 部分可见时的定位策略

### 22.1 只有运动变化带，没有可靠边线

输出：

```text
geometry_state = PARTIAL_ENTERING
pose_detected = false
pose_valid_for_grasp = false
```

只显示候选框和“箱体正在进入视野”。

### 22.2 只检测到一条长边

可获得：

- 边线方向；
- 大致 yaw；
- 可能获得内部法向。

不能唯一确定沿箱体长度方向的完整中心。

输出：

```text
geometry_state = PARTIAL_GEOMETRY
```

可以提供局部观察点，但必须明确它不是完整箱体中心。

### 22.3 检测到左侧短边和一条长边

这是部分进入时最有价值的情况。

已知：

- 左侧短边；
- 一条长边；
- 长度 `L`；
- 宽度 `W`；
- 长度内部方向；
- 宽度内部方向。

若获得角点 `corner`：

```text
center_uv =
    corner
    + length_inward * L / 2
    + width_inward * W / 2
```

再转换回相机三维：

```cpp
Eigen::Vector3f center_camera =
    unprojectFromPlane(
        center_uv,
        plane_basis);
```

输出：

```text
geometry_state = ESTIMATED_CENTER
```

这是基于已知尺寸推算的观察中心，仍然禁止抓取。

### 22.4 检测到完整左侧短边中心

```text
center_camera =
    front_edge_center_camera
    + box_length_inward_direction
      * outer_length / 2
```

滚道自右向左运动：

- 运动方向指向左；
- 箱体内部长度方向通常指向右；
- 两者方向相反。

必须在局部平面中使用明确的三维方向向量，不能仅依赖图像左右符号。

### 22.5 检测到两条长边

可以确定：

- 横向中心线；
- yaw；
- 宽度。

若没有短边或前沿，长度中心仍不可靠。

输出：

```text
PARTIAL_GEOMETRY
```

### 22.6 两条长边 + 左侧短边

可以较可靠地推算完整观察中心。

输出：

```text
ESTIMATED_CENTER
```

若凹槽、尺寸、平行和垂直约束均充分，可提升为：

```text
SUFFICIENT_GEOMETRY
```

### 22.7 大部分矩形可见

输出：

```text
SUFFICIENT_GEOMETRY
```

但传送带仍在运动，因此：

```cpp
pose_valid_for_grasp = false;
```

---

## 23. 运动参考位置

禁止优先使用可见点云质心。

随着箱体从右侧进入视野：

```text
可见点云质心变化
=
真实运动
+
可见范围增加造成的伪变化
```

优先跟踪左侧前沿。

设局部平面中的运动方向单位向量：

```cpp
Eigen::Vector2f motion_direction_uv;
```

其方向定义为箱体向左的实际运动方向。

对点：

```text
s_i = motion_direction_uv dot point_uv
```

前沿取高分位数：

```text
s_front = percentile(s_i, 0.95)
```

若方向定义相反，则取低分位数。

推荐：

```yaml
front_edge:
  percentile: 0.95
  band_m: 0.020
  min_points: 20
```

前沿位置比可见点云中心更适合做短窗口运动估计。

---

## 24. 运动估计

输入：

```cpp
struct MotionObservation
{
    double timestamp_sec = 0.0;
    float reference_position_m = 0.0F;
    float confidence = 0.0F;
};
```

输出：

```cpp
struct MotionEstimate
{
    bool valid = false;

    float velocity_mps = 0.0F;
    float position_span_m = 0.0F;
    float fit_rmse_m = 0.0F;
    float confidence = 0.0F;

    MotionState state =
        MotionState::UNKNOWN;
};
```

拟合：

```text
s(t) = v * t + b
```

本阶段测速作用：

```text
判断箱体仍在运动
```

而不是：

```text
预测人工停止后的最终位置
```


---

## 25. 运动状态判定

建议同时使用：

1. 速度绝对值；
2. 采样窗口内位置跨度；
3. 速度拟合 RMSE；
4. 有效前沿观测数量。

```cpp
MotionState classifyMotion(
    const MotionEstimate& estimate,
    const MotionConfig& config)
{
    if (!estimate.valid)
    {
        return MotionState::UNKNOWN;
    }

    if (estimate.confidence <
        config.min_confidence)
    {
        return MotionState::UNKNOWN;
    }

    if (std::abs(
            estimate.velocity_mps) >=
        config.
            moving_velocity_threshold_mps)
    {
        return MotionState::MOVING;
    }

    if (estimate.position_span_m >=
        config.
            moving_position_span_threshold_m)
    {
        return MotionState::MOVING;
    }

    if (std::abs(
            estimate.velocity_mps) <=
            config.
                static_velocity_threshold_mps &&
        estimate.position_span_m <=
            config.
                static_position_span_threshold_m)
    {
        return MotionState::
            STATIC_CANDIDATE;
    }

    return MotionState::UNKNOWN;
}
```

推荐初值：

```yaml
motion_estimator:
  min_valid_frames: 6
  moving_velocity_threshold_mps: 0.020
  moving_position_span_threshold_m: 0.015
  static_velocity_threshold_mps: 0.010
  static_position_span_threshold_m: 0.008
  max_fit_rmse_m: 0.015
  min_confidence: 0.65
```

---

## 26. 单帧几何估计流程

```text
当前完整深度图
    ↓
使用扩张后的候选框
    ↓
生成相机坐标系三维点云
    ↓
相机三维 ROI 过滤
    ↓
提取上沿候选点
    ↓
三维平面 RANSAC
    ↓
构造上沿局部平面基底
    ↓
投影上沿点到二维米制平面
    ↓
二维 RANSAC 拟合最多四条边
    ↓
边线分类
    ↓
凹槽验证
    ↓
视野边界和可见完整度判断
    ↓
选择部分可见定位策略
    ↓
输出 GeometryEstimate
```

---

## 27. 上沿候选点提取

这是方案中最难调试的部分之一。

建议第一版组合以下条件，而不是只依赖单一“高度”：

### 27.1 候选框图像上部

保留候选框较上部分：

```yaml
rim_candidate:
  image_top_ratio: 0.65
```

这只是弱先验，因为相机倾斜安装后，图像上下不完全等于物理高度。

### 27.2 靠近相机的深度分位层

对候选点的 `Zcamera` 取较近分位数：

```text
near_depth = percentile(Zcamera, 0.25)
```

保留靠近该深度层的点。

该条件也只能作为弱先验。

### 27.3 深度梯度

上沿附近通常存在明显深度梯度。

可以对 `CV_16UC1` 深度图转换为 `CV_32F` 后计算 Sobel：

```cpp
cv::Sobel(
    depth_f32,
    grad_x,
    CV_32F,
    1,
    0,
    3);

cv::Sobel(
    depth_f32,
    grad_y,
    CV_32F,
    0,
    1,
    3);

cv::magnitude(
    grad_x,
    grad_y,
    gradient);
```

深度无效点附近会出现伪梯度，需要先构建有效深度掩膜并适当腐蚀。

### 27.4 局部平面性

后续可对三维邻域做 PCA：

```text
lambda0 <= lambda1 <= lambda2
```

平面性较强时：

```text
lambda0 / (lambda0 + lambda1 + lambda2)
```

较小。

第一版可以暂不实现局部法向，只用：

```text
候选框上部
+
深度梯度
+
靠近相机分位层
```

生成上沿候选点。

---

## 28. PlaneRansac3D 设计

### 28.1 接口

```cpp
class PlaneRansac3D
{
public:
    std::optional<Plane3D> fit(
        const std::vector<
            Eigen::Vector3f>& points,
        int max_iterations,
        float inlier_distance_m,
        int min_inliers) const;
};
```

### 28.2 随机三点生成平面

```cpp
bool makePlaneFromThreePoints(
    const Eigen::Vector3f& p0,
    const Eigen::Vector3f& p1,
    const Eigen::Vector3f& p2,
    Plane3D& plane)
{
    const Eigen::Vector3f v1 =
        p1 - p0;

    const Eigen::Vector3f v2 =
        p2 - p0;

    Eigen::Vector3f normal =
        v1.cross(v2);

    const float norm =
        normal.norm();

    if (norm < 1e-6F)
    {
        return false;
    }

    normal /= norm;

    plane.normal = normal;
    plane.d = -normal.dot(p0);

    return true;
}
```

### 28.3 最优内点精拟合

RANSAC 找到最佳内点集合后，使用所有内点 PCA 精拟合平面。

对内点计算中心和协方差矩阵。

最小特征值对应的特征向量作为平面法向。

### 28.4 法向符号

平面法向有正负二义性。

可规定法向大致朝向相机或远离相机，以保持连续。

例如：

```cpp
if (plane.normal.dot(
        plane.centroid) > 0.0F)
{
    plane.normal =
        -plane.normal;

    plane.d =
        -plane.d;
}
```

符号规则需要结合相机坐标含义验证。

---

## 29. LineRansac2D 设计

### 29.1 接口

```cpp
class LineRansac2D
{
public:
    std::optional<Line2D> fit(
        const std::vector<
            Eigen::Vector2f>& points,
        int max_iterations,
        float inlier_distance_m,
        int min_inliers) const;

    std::vector<Line2D> fitMultiple(
        const std::vector<
            Eigen::Vector2f>& points,
        int maximum_line_count) const;
};
```

### 29.2 候选线生成

```cpp
bool makeLineFromTwoPoints(
    const Eigen::Vector2f& p0,
    const Eigen::Vector2f& p1,
    Line2D& line)
{
    Eigen::Vector2f direction =
        p1 - p0;

    const float norm =
        direction.norm();

    if (norm < 1e-6F)
    {
        return false;
    }

    direction /= norm;

    line.point = p0;
    line.direction = direction;
    line.normal =
        Eigen::Vector2f(
            -direction.y(),
            direction.x());

    return true;
}
```

### 29.3 内点精拟合

对内点做二维 PCA。

最大特征值方向为直线方向。

线段可见长度：

```text
t_i =
    direction dot
    (point_i - centroid)
```

```text
visible_length =
    max(t_i) - min(t_i)
```

### 29.4 多条边提取

```text
拟合第一条线
    ↓
删除第一条线内点
    ↓
拟合第二条线
    ↓
继续，最多四条
```

删除内点时可适当保留交角附近的点，避免角点被第一条边全部吃掉。

第一版可以直接删除，后续再优化。

---

## 30. 长边和短边分类

不能只根据可见线段长度判断长短边。

应组合：

1. 已知箱体长宽；
2. 多线之间的平行关系；
3. 多线之间的垂直关系；
4. 可见线段长度；
5. 自右向左运动方向；
6. 凹槽内侧方向；
7. 箱体通常摆放方向。

结构：

```cpp
struct ClassifiedEdges
{
    std::optional<Line2D>
        front_short_edge;

    std::optional<Line2D>
        back_short_edge;

    std::optional<Line2D>
        long_edge_a;

    std::optional<Line2D>
        long_edge_b;
};
```

两条线夹角：

```cpp
float angleBetweenDirections(
    const Eigen::Vector2f& a,
    const Eigen::Vector2f& b)
{
    const float cosine =
        std::clamp(
            std::abs(
                a.normalized().dot(
                    b.normalized())),
            0.0F,
            1.0F);

    return std::acos(cosine);
}
```

使用绝对点积后，平行线角度接近 0。

垂直线角度接近 `pi / 2`。

---

## 31. 角点计算

二维直线：

```text
p1 + t d1
p2 + s d2
```

求解：

```cpp
std::optional<Eigen::Vector2f>
intersectLines(
    const Line2D& line_a,
    const Line2D& line_b)
{
    Eigen::Matrix2f A;

    A.col(0) =
        line_a.direction;

    A.col(1) =
        -line_b.direction;

    const float determinant =
        A.determinant();

    if (std::abs(
            determinant) < 1e-6F)
    {
        return std::nullopt;
    }

    const Eigen::Vector2f b =
        line_b.point -
        line_a.point;

    const Eigen::Vector2f solution =
        A.fullPivLu().solve(b);

    return
        line_a.point +
        solution.x() *
            line_a.direction;
}
```

---

## 32. 凹槽验证实现

### 32.1 点到上沿平面的有符号距离

```cpp
float signedDistanceToPlane(
    const Eigen::Vector3f& point,
    const Plane3D& plane)
{
    return
        plane.normal.dot(point) +
        plane.d;
}
```

需要统一法向符号，保证“向箱体内部下方”为固定正或负方向。

### 32.2 两侧内部判断

对某条边线，在二维平面中构建：

```text
positive normal side
negative normal side
```

分别统计对应三维点低于上沿平面的程度。

```cpp
struct SideCavityScore
{
    int valid_point_count = 0;
    int low_point_count = 0;

    float mean_drop_m = 0.0F;
    float score = 0.0F;
};
```

凹槽更明显的一侧定义为内部方向。

### 32.3 箱内有物品

箱内有物品时，不能要求整个内部区域都很低。

建议：

- 使用低点比例；
- 使用中位高度差；
- 只要求部分区域存在下凹；
- 不要求内腔底部完整可见。

---

## 33. 几何状态分类

建议按从高到低的顺序判断：

```cpp
GeometryState classifyGeometryState(
    const GeometryEstimate& geometry)
{
    if (!geometry.valid &&
        geometry.visibility.
            touches_right_border)
    {
        return GeometryState::
            PARTIAL_ENTERING;
    }

    if (geometry.cavity_verified &&
        geometry.front_short_edge_visible &&
        geometry.long_edge_a_visible &&
        geometry.long_edge_b_visible)
    {
        return GeometryState::
            SUFFICIENT_GEOMETRY;
    }

    if (geometry.front_short_edge_visible &&
        (geometry.long_edge_a_visible ||
         geometry.long_edge_b_visible))
    {
        return GeometryState::
            ESTIMATED_CENTER;
    }

    if (geometry.long_edge_a_visible ||
        geometry.long_edge_b_visible)
    {
        return GeometryState::
            PARTIAL_GEOMETRY;
    }

    if (geometry.visibility.
            touches_right_border)
    {
        return GeometryState::
            PARTIAL_ENTERING;
    }

    return GeometryState::
        GEOMETRY_FAILED;
}
```

---

## 34. BoxGeometryEstimator 接口

```cpp
class BoxGeometryEstimator
{
public:
    GeometryEstimate estimate(
        const std::vector<
            PointSample>& points,
        const cv::Rect& candidate_bbox,
        const cv::Size& image_size,
        const BoxModel& box_model) const;

private:
    std::vector<
        Eigen::Vector3f>
    extractRimCandidates(
        const std::vector<
            PointSample>& points,
        const DepthFrame& frame,
        const cv::Rect& bbox) const;

    GeometryEstimate
    estimatePartialGeometry(
        const Plane3D& rim_plane,
        const PlaneBasis& basis,
        const std::vector<
            Eigen::Vector3f>& rim_points,
        const std::vector<
            Eigen::Vector3f>& candidate_points,
        const BoxModel& box_model,
        const VisibilityEstimate&
            visibility) const;
};
```

---

## 35. 几何估计主逻辑伪代码

```cpp
GeometryEstimate
BoxGeometryEstimator::estimate(
    const std::vector<
        PointSample>& point_samples,
    const cv::Rect& candidate_bbox,
    const cv::Size& image_size,
    const BoxModel& box_model) const
{
    GeometryEstimate result;

    result.visibility =
        estimateVisibility(
            candidate_bbox,
            image_size,
            config_.
                border_margin_px);

    if (point_samples.size() <
        static_cast<std::size_t>(
            config_.
                min_candidate_points))
    {
        if (result.visibility.
                touches_right_border)
        {
            result.geometry_state =
                GeometryState::
                    PARTIAL_ENTERING;
        }

        return result;
    }

    const auto rim_candidates =
        extractRimCandidates(
            point_samples);

    if (rim_candidates.size() <
        static_cast<std::size_t>(
            config_.
                min_rim_candidate_points))
    {
        if (result.visibility.
                touches_right_border)
        {
            result.geometry_state =
                GeometryState::
                    PARTIAL_ENTERING;
        }

        return result;
    }

    const auto plane_optional =
        plane_ransac_.fit(
            rim_candidates,
            config_.
                plane_max_iterations,
            config_.
                plane_inlier_distance_m,
            config_.
                plane_min_inliers);

    if (!plane_optional)
    {
        result.geometry_state =
            result.visibility.
                touches_right_border
            ? GeometryState::
                PARTIAL_ENTERING
            : GeometryState::
                GEOMETRY_FAILED;

        return result;
    }

    const Plane3D rim_plane =
        *plane_optional;

    const PlaneBasis basis =
        makePlaneBasis(
            rim_plane);

    std::vector<Eigen::Vector2f>
        rim_points_uv;

    rim_points_uv.reserve(
        rim_plane.
            inlier_indices.size());

    for (const std::size_t index :
         rim_plane.inlier_indices)
    {
        rim_points_uv.push_back(
            projectToPlane(
                rim_candidates[index],
                basis));
    }

    const auto lines =
        line_ransac_.fitMultiple(
            rim_points_uv,
            4);

    const ClassifiedEdges edges =
        classifyEdges(
            lines,
            box_model);

    const CavityEstimate cavity =
        evaluateCavity(
            point_samples,
            rim_plane,
            basis,
            edges,
            box_model);

    result =
        estimateFromVisibleEdges(
            rim_plane,
            basis,
            edges,
            cavity,
            result.visibility,
            box_model);

    return result;
}
```

---

## 36. 不同边线组合的中心推算

### 36.1 一条短边 + 一条长边

```cpp
std::optional<Eigen::Vector2f>
estimateCenterFromCorner(
    const Line2D& short_edge,
    const Line2D& long_edge,
    const Eigen::Vector2f&
        length_inward,
    const Eigen::Vector2f&
        width_inward,
    const BoxModel& model)
{
    const auto corner =
        intersectLines(
            short_edge,
            long_edge);

    if (!corner)
    {
        return std::nullopt;
    }

    return
        *corner +
        length_inward.normalized() *
            (model.outer_length_m *
             0.5F) +
        width_inward.normalized() *
            (model.outer_width_m *
             0.5F);
}
```

### 36.2 完整短边中心

```cpp
center_uv =
    front_short_edge_center_uv +
    length_inward.normalized() *
        (model.outer_length_m *
         0.5F);
```

### 36.3 两条长边

两条平行长边中间线可确定横向中心线。

但没有短边或稳定前沿时，不得声称长度中心已确定。

---

## 37. 几何置信度

可以组合：

```text
平面内点比例
平面 RMSE
边线数量
边线长度
直线 RMSE
平行/垂直一致性
凹槽分数
尺寸一致性
是否接触右边界
```

示例：

```cpp
float computeGeometryConfidence(
    GeometryState state,
    float plane_rmse_m,
    float line_rmse_m,
    float cavity_score,
    bool touches_right_border)
{
    float state_score = 0.0F;

    switch (state)
    {
    case GeometryState::
        SUFFICIENT_GEOMETRY:
        state_score = 1.0F;
        break;

    case GeometryState::
        ESTIMATED_CENTER:
        state_score = 0.80F;
        break;

    case GeometryState::
        PARTIAL_GEOMETRY:
        state_score = 0.55F;
        break;

    case GeometryState::
        PARTIAL_ENTERING:
        state_score = 0.30F;
        break;

    default:
        state_score = 0.0F;
        break;
    }

    const float plane_score =
        std::clamp(
            1.0F -
            plane_rmse_m / 0.025F,
            0.0F,
            1.0F);

    const float line_score =
        std::clamp(
            1.0F -
            line_rmse_m / 0.025F,
            0.0F,
            1.0F);

    float confidence =
        0.35F * state_score +
        0.20F * plane_score +
        0.20F * line_score +
        0.25F * cavity_score;

    if (touches_right_border)
    {
        confidence *= 0.80F;
    }

    return std::clamp(
        confidence,
        0.0F,
        1.0F);
}
```

---

## 38. 多帧候选关联

同一次检测窗口内，应尽量把多帧中的候选关联为同一个箱体。

第一版假设视野内最多一个主要运动箱体时，可以选择：

- 面积最大的运动候选；
- 与上一帧前沿位置最近；
- 深度范围相似；
- 候选框重叠最大。

后续多箱体时再增加轨迹 ID。

---

## 39. DetectionService 接口

```cpp
class DetectionService
{
public:
    DetectionResult detectOnce();

private:
    D435FrameProvider&
        frame_provider_;

    MotionMaskBuilder&
        motion_mask_builder_;

    CandidateExtractor&
        candidate_extractor_;

    PointCloudBuilder&
        point_cloud_builder_;

    BoxGeometryEstimator&
        geometry_estimator_;

    MotionEstimator&
        motion_estimator_;

    BoxModel box_model_;

    Eigen::Isometry3f
        T_base_camera_ =
            Eigen::Isometry3f::
                Identity();
};
```

---

## 40. DetectionService 主流程

```cpp
DetectionResult
DetectionService::detectOnce()
{
    DetectionResult result;

    std::vector<DepthFrame> frames;

    collectDepthFrames(
        config_.capture_window_sec,
        frames);

    result.total_frame_count =
        static_cast<int>(
            frames.size());

    if (frames.size() <
        static_cast<std::size_t>(
            config_.
                minimum_capture_frames))
    {
        result.status =
            DetectionStatus::
                DETECTION_FAILED;

        result.pose_valid_for_grasp =
            false;

        result.message =
            "深度帧数量不足，禁止抓取。";

        return result;
    }

    const cv::Mat motion_mask =
        motion_mask_builder_.build(
            frames);

    const auto candidates =
        candidate_extractor_.extract(
            motion_mask);

    if (candidates.empty())
    {
        result.status =
            DetectionStatus::NO_BOX;

        result.box_detected = false;
        result.pose_detected = false;
        result.pose_valid_for_grasp =
            false;

        result.message =
            "未检测到可靠运动箱体。";

        return result;
    }

    std::vector<BoxObservation>
        observations;

    for (const DepthFrame& frame :
         frames)
    {
        const auto candidate =
            selectCandidateForFrame(
                frame,
                candidates);

        if (!candidate)
        {
            continue;
        }

        const cv::Rect expanded_bbox =
            expandCandidateRightward(
                candidate->bbox,
                frame.depth_u16.size(),
                config_);

        auto points =
            point_cloud_builder_.build(
                frame,
                expanded_bbox);

        points =
            filterCameraRoi(
                points,
                config_.camera_roi_3d);

        GeometryEstimate geometry =
            geometry_estimator_.estimate(
                points,
                expanded_bbox,
                frame.depth_u16.size(),
                box_model_);

        BoxObservation observation;

        observation.timestamp_sec =
            frame.timestamp_sec;

        observation.image_bbox =
            expanded_bbox;

        observation.geometry =
            geometry;

        observation.valid =
            geometry.valid ||
            geometry.geometry_state ==
                GeometryState::
                    PARTIAL_ENTERING;

        observation.detection_confidence =
            combineDetectionConfidence(
                geometry);

        if (observation.valid)
        {
            observations.push_back(
                observation);
        }
    }

    result.valid_frame_count =
        static_cast<int>(
            observations.size());

    if (observations.empty())
    {
        result.status =
            DetectionStatus::
                MOTION_UNCERTAIN;

        result.box_detected = true;
        result.pose_detected = false;
        result.pose_valid_for_grasp =
            false;

        result.message =
            "检测到运动变化，但几何信息不足，禁止抓取。";

        return result;
    }

    std::vector<MotionObservation>
        motion_observations;

    for (const BoxObservation&
         observation :
         observations)
    {
        if (!observation.geometry.
                motion_reference_valid)
        {
            continue;
        }

        motion_observations.push_back(
            MotionObservation{
                observation.timestamp_sec,
                observation.geometry.
                    motion_reference_m,
                observation.
                    detection_confidence});
    }

    const MotionEstimate motion =
        motion_estimator_.estimate(
            motion_observations);

    result.motion_state =
        motion.state;

    result.estimated_velocity_mps =
        motion.velocity_mps;

    result.position_span_m =
        motion.position_span_m;

    result.velocity_fit_rmse_m =
        motion.fit_rmse_m;

    result.motion_confidence =
        motion.confidence;

    const GeometryEstimate
        best_geometry =
            selectBestGeometryObservation(
                observations);

    result.geometry_state =
        best_geometry.geometry_state;

    result.geometry_confidence =
        best_geometry.
            geometry_confidence;

    result.box_detected = true;

    if (best_geometry.valid)
    {
        result.observation_pose =
            best_geometry.
                observation_pose;

        result.observation_pose.
            position_base =
            T_base_camera_ *
            result.observation_pose.
                position_camera;

        result.pose_detected = true;
    }

    result.latest_sensor_timestamp_sec =
        observations.back().
            timestamp_sec;

    result.status =
        combineStatus(
            result.motion_state,
            result.geometry_state);

    // 第一阶段硬性安全约束。
    result.pose_valid_for_grasp =
        false;

    result.message =
        buildStatusMessage(result);

    return result;
}
```

---

## 41. 状态组合

```cpp
DetectionStatus combineStatus(
    MotionState motion_state,
    GeometryState geometry_state)
{
    if (motion_state ==
        MotionState::UNKNOWN)
    {
        return DetectionStatus::
            MOTION_UNCERTAIN;
    }

    if (motion_state ==
        MotionState::
            STATIC_CANDIDATE)
    {
        return DetectionStatus::
            STATIC_CANDIDATE;
    }

    switch (geometry_state)
    {
    case GeometryState::
        PARTIAL_ENTERING:
        return DetectionStatus::
            MOVING_PARTIAL_ENTERING;

    case GeometryState::
        PARTIAL_GEOMETRY:
        return DetectionStatus::
            MOVING_PARTIAL_GEOMETRY;

    case GeometryState::
        ESTIMATED_CENTER:
        return DetectionStatus::
            MOVING_ESTIMATED_CENTER;

    case GeometryState::
        SUFFICIENT_GEOMETRY:
        return DetectionStatus::
            MOVING_SUFFICIENT_GEOMETRY;

    default:
        return DetectionStatus::
            MOTION_UNCERTAIN;
    }
}
```

---

## 42. 状态提示语

```text
MOVING_PARTIAL_ENTERING
    检测到箱体正在从右侧进入视野，
    当前仅看到左侧部分，
    几何信息不足，禁止抓取。

MOVING_PARTIAL_GEOMETRY
    检测到运动箱体及部分边线，
    当前观察位姿不完整，禁止抓取。

MOVING_ESTIMATED_CENTER
    已根据可见短边、长边和已知尺寸推算观察中心，
    箱体仍在运动，禁止抓取。

MOVING_SUFFICIENT_GEOMETRY
    当前帧箱体几何信息较充分，
    但箱体仍在运动，
    当前位姿仅用于观察，禁止抓取。

MOTION_UNCERTAIN
    检测到运动变化，
    但运动或几何置信度不足，禁止抓取。

STATIC_CANDIDATE
    箱体疑似静止，
    但尚未完成停止后的静态精定位，禁止抓取。
```

---

## 43. 推荐配置文件

```yaml
camera:
  depth_width: 640
  depth_height: 480
  fps: 30
  frame_timeout_ms: 100

capture:
  window_sec: 0.80
  minimum_capture_frames: 15
  minimum_valid_geometry_frames: 5

motion_mask:
  frame_gap: 5
  depth_difference_threshold_m: 0.025
  min_motion_votes: 3
  morphology_kernel_size: 5

candidate:
  min_motion_pixels: 300
  max_motion_pixels: 180000
  min_bbox_width_px: 10
  min_bbox_height_px: 20
  max_candidates: 3

candidate_expand:
  left_px: 20
  right_px: 250
  up_px: 50
  down_px: 80

visibility:
  border_margin_px: 10

camera_roi_3d:
  x_min_m: -0.80
  x_max_m: 0.80
  y_min_m: -0.50
  y_max_m: 1.20
  z_min_m: 0.30
  z_max_m: 2.00

point_cloud:
  min_candidate_points: 300
  sampling_step_px: 1

rim_candidate:
  image_top_ratio: 0.65
  near_depth_percentile: 0.25
  depth_gradient_threshold_raw: 30
  min_rim_candidate_points: 80

plane_ransac:
  max_iterations: 300
  inlier_distance_m: 0.012
  min_inliers: 60
  min_plane_extent_m: 0.10

line_ransac:
  max_iterations: 200
  inlier_distance_m: 0.012
  min_inliers: 25
  min_visible_length_m: 0.10
  parallel_tolerance_deg: 8.0
  perpendicular_tolerance_deg: 10.0

cavity:
  min_cavity_drop_m: 0.040
  min_inner_points_partial: 20
  min_inner_points_confirmed: 80
  min_cavity_score_partial: 0.20
  min_cavity_score_confirmed: 0.35

front_edge:
  percentile: 0.95
  band_m: 0.020
  min_points: 20

motion_estimator:
  min_valid_frames: 6
  moving_velocity_threshold_mps: 0.020
  moving_position_span_threshold_m: 0.015
  static_velocity_threshold_mps: 0.010
  static_position_span_threshold_m: 0.008
  max_fit_rmse_m: 0.015
  min_confidence: 0.65

box_model:
  outer_length_m: 0.600
  outer_width_m: 0.400
  outer_height_m: 0.280
  rim_thickness_m: 0.025
  inner_length_m: 0.550
  inner_width_m: 0.350
  cavity_depth_m: 0.220

output:
  always_disable_grasp_in_mvp: true
```

所有参数仅为起始值，必须根据现场数据调试。


---

## 44. 主控使用规范

主控只能依据：

```cpp
result.pose_valid_for_grasp
```

决定是否允许抓取。

第一阶段：

```cpp
if (result.pose_valid_for_grasp)
{
    // MVP 阶段不应进入。
    executeGrasp(
        result.observation_pose);
}
else
{
    disableGraspCommand();
}
```

禁止：

```cpp
if (result.box_detected)
{
    executeGrasp(...);
}
```

禁止：

```cpp
if (result.pose_detected)
{
    executeGrasp(...);
}
```

`box_detected` 只表示存在箱体候选。

`pose_detected` 只表示视觉产生了当前观察位姿。

它们都不是抓取许可。

---

## 45. 调试输出

建议每次请求保存：

```text
01_depth_first.png
02_depth_last.png
03_pair_motion_mask.png
04_motion_vote_map.png
05_final_motion_mask.png
06_motion_components.png
07_expanded_candidate.png
08_candidate_depth.png
09_rim_candidate_pixels.png
10_plane_inliers.png
11_plane_projection_uv.png
12_ransac_lines_uv.png
13_cavity_points_uv.png
14_estimated_box_rectangle_uv.png
15_observation_pose.png
16_result.json
```

局部平面二维调试图中建议绘制：

```text
长边
短边
凹槽低点
推算出的箱体外框
推算中心
左侧运动前沿
候选是否接触右边界
```

调试颜色仅用于显示，不参与检测。

---

## 46. 日志格式

每次主控请求至少记录：

```csv
request_time,
status,
motion_state,
geometry_state,
total_frames,
valid_frames,
estimated_velocity_mps,
position_span_m,
velocity_fit_rmse_m,
motion_confidence,
geometry_confidence,
touches_right_border,
front_short_edge_visible,
long_edge_count,
cavity_score,
pose_detected,
pose_x_camera,
pose_y_camera,
pose_z_camera,
pose_x_base,
pose_y_base,
pose_z_base,
yaw,
pose_valid_for_grasp
```

还应记录主要参数版本，避免后续无法复现实验结果。

---

## 47. 深度序列录制与回放

视觉算法的主要验证方式应是现场深度数据回放。

### 47.1 必须录制的场景

1. 无箱体；
2. 箱体刚从右侧进入，只看到左侧很窄一部分；
3. 看到左侧短边，但长边很短；
4. 看到左侧短边和一条长边；
5. 只看到一条长边；
6. 看到两条长边但没有短边；
7. 两条长边加左侧短边；
8. 大部分箱体进入视野；
9. 空箱；
10. 箱内有物品；
11. 栏杆干扰；
12. 深度孔洞；
13. 机器人站位略左；
14. 机器人站位略右；
15. 机器人距离传送带略近；
16. 机器人距离传送带略远；
17. 人工停止前后；
18. 滚筒旋转造成局部深度噪声。

### 47.2 保存格式

```text
sequence_name/
├── depth_000001.png
├── depth_000002.png
├── ...
├── timestamps.csv
├── camera_intrinsics.yaml
└── metadata.yaml
```

`metadata.yaml` 示例：

```yaml
scene_name: partial_entering_from_right
box_present: true
expected_motion: moving
expected_geometry: partial_entering
robot_position_note: slightly_left
contains_object_inside_box: false
```

---

## 48. 普通 C++ 测试程序

不使用 GoogleTest。

示例：

```cpp
int main()
{
    MotionEstimator estimator(config);

    std::vector<MotionObservation>
        observations = {
            {0.0, 0.80F, 1.0F},
            {0.1, 0.78F, 1.0F},
            {0.2, 0.76F, 1.0F},
            {0.3, 0.74F, 1.0F}
        };

    const MotionEstimate result =
        estimator.estimate(
            observations);

    if (result.state !=
        MotionState::MOVING)
    {
        std::cerr
            << "FAILED: expected MOVING"
            << std::endl;

        return 1;
    }

    std::cout
        << "PASSED"
        << std::endl;

    return 0;
}
```

也可使用：

```cpp
assert(
    result.state ==
    MotionState::MOVING);
```

---

## 49. 需要验证的纯算法场景

### 49.1 PlaneRansac3D

构造：

- 一个带噪声的三维平面；
- 20%～40% 离群点；
- 三个点共线；
- 点数不足；
- 两个相近平面。

检查：

- 法向误差；
- 平面中心误差；
- RMSE；
- 内点数量。

### 49.2 LineRansac2D

构造：

- 水平线；
- 斜线；
- 30% 离群点；
- 两条相交线；
- 两条平行线；
- 短线段；
- 点数不足。

检查：

- 方向误差；
- 线段可见长度；
- 内点数量；
- RMSE。

### 49.3 一条短边 + 一条长边中心推算

已知：

```text
corner
length_inward
width_inward
L
W
```

检查：

```text
center =
    corner
    + length_inward * L/2
    + width_inward * W/2
```

### 49.4 凹槽内部方向

构造一条边线，两侧分别放置：

- 一侧上沿同高点；
- 一侧明显下凹点。

检查算法能正确选择凹槽侧作为内部方向。

### 49.5 视野裁剪

模拟箱体逐步从右侧进入。

检查：

- 候选接触右边界时为 `PARTIAL_ENTERING`；
- 可见点云质心发生变化不会被当作真实中心；
- 左侧前沿保持连续；
- 看到短边和长边后转为 `ESTIMATED_CENTER`。

---

## 50. 推荐开发顺序

### 第1步：工程骨架

实现：

- CMake；
- C++17；
- types.hpp；
- yaml-cpp；
- 基础日志；
- 普通测试程序。

### 第2步：D435FrameProvider

实现：

- 相机常驻；
- 深度帧 clone；
- 时间戳；
- 内参；
- depth scale；
- 掉帧检查。

### 第3步：深度序列录制与回放

应尽早完成。

后续算法开发尽量基于录制数据，减少频繁占用现场设备。

### 第4步：MotionMaskBuilder

实现：

- 间隔帧差分；
- 多组投票；
- 有效深度掩膜；
- 形态学；
- 调试图。

### 第5步：CandidateExtractor

实现：

- 连通域；
- 面积筛选；
- 边界接触；
- 候选框向右扩张。

### 第6步：PointCloudBuilder

实现：

- 当前完整深度图反投影；
- 相机坐标点云；
- 相机三维 ROI。

### 第7步：PlaneRansac3D

实现：

- 三点平面；
- 点到平面距离；
- 内点统计；
- 最优内点重新拟合；
- RMSE。

### 第8步：PlaneBasis 和 LineRansac2D

实现：

- 三维到二维；
- 二维到三维；
- 多条直线；
- 可见长度；
- 平行与垂直关系。

### 第9步：凹槽验证

实现：

- 点到上沿平面的距离；
- 边线两侧统计；
- 内部方向；
- partial 和 confirmed 两级阈值。

### 第10步：部分可见定位

依次实现：

1. `PARTIAL_ENTERING`；
2. 一条长边；
3. 左侧短边 + 一条长边；
4. 两条长边；
5. 两条长边 + 左侧短边；
6. 大部分矩形可见。

### 第11步：MotionEstimator

使用左侧前沿。

不要使用可见点云质心作为首选运动参考。

### 第12步：DetectionService

完成：

- 状态组合；
- 相机坐标；
- base 外参变换；
- 日志；
- 调试图；
- 硬性禁止抓取。

---

## 51. 给 Codex 的第一轮任务

```text
请使用 C++17 创建 box_vision 工程骨架。

依赖：
- librealsense2
- OpenCV
- Eigen3
- yaml-cpp
- CMake

不要使用 GoogleTest。
不要建立传送带坐标系。

第一轮实现：

1. types.hpp
   - MotionState
   - GeometryState
   - DetectionStatus
   - DepthFrame
   - Pose3D
   - VisibilityEstimate
   - GeometryEstimate
   - BoxObservation
   - DetectionResult

2. D435FrameProvider
   - 相机常驻运行
   - 输出 CV_16UC1 深度图
   - 必须 clone
   - 输出真实时间戳
   - 输出内参
   - 输出 depth scale

3. 深度序列录制工具
   - 保存 PNG 深度图
   - 保存 timestamps.csv
   - 保存 camera_intrinsics.yaml
   - 保存 metadata.yaml

4. 深度序列回放工具
   - 按时间戳顺序读取
   - 构造 DepthFrame

5. 所有 DetectionResult 默认：
   pose_valid_for_grasp = false

6. 提供普通 C++ 测试程序，
   不使用 GoogleTest。
```

---

## 52. 给 Codex 的第二轮任务

```text
实现 MotionMaskBuilder 和 CandidateExtractor。

要求：

1. 输入 0.8 秒连续深度帧。
2. 使用间隔帧差分。
3. 不使用长期固定背景。
4. 默认 frame_gap = 5。
5. 只比较两帧都有效的深度像素。
6. 深度差阈值可配置。
7. 在整个窗口累计运动票数。
8. 使用闭运算、开运算和可选膨胀。
9. 使用 connectedComponentsWithStats。
10. 过滤小连通域。
11. 箱体自右向左运动：
    - 候选框主要向右扩张；
    - 候选接触图像右边界时，
      设置 touches_right_border = true。
12. 输出完整调试图。
```

---

## 53. 给 Codex 的第三轮任务

```text
实现 PointCloudBuilder、PlaneRansac3D 和 PlaneBasis。

要求：

1. 从当前完整深度图中提取扩张候选框内的点。
2. 使用 rs2_deproject_pixel_to_point。
3. 全部点保持在 camera_depth_optical_frame。
4. 使用相机坐标系三维 ROI。
5. 不建立传送带坐标系。
6. 实现 PlaneRansac3D：
   - 随机三个不共线点；
   - 生成平面；
   - 点到平面距离；
   - 统计内点；
   - 最优内点重新拟合；
   - 输出法向、中心和 RMSE。
7. 构造 PlaneBasis：
   - origin_camera
   - axis_u_camera
   - axis_v_camera
   - normal_camera
8. 实现三维点投影到局部二维米制坐标。
9. 实现二维坐标还原到相机三维坐标。
```

---

## 54. 给 Codex 的第四轮任务

```text
实现 LineRansac2D 和部分箱体几何估计。

要求：

1. LineRansac2D 输入是局部平面中的米制二维点，
   不是图像像素。
2. 最多拟合四条边。
3. 输出每条线：
   - point
   - direction
   - normal
   - inliers
   - visible_length_m
   - rmse_m
4. 根据已知箱体尺寸、平行和垂直关系分类长短边。
5. 实现凹槽点分布判断箱体内部方向。
6. 实现：
   - PARTIAL_ENTERING
   - PARTIAL_GEOMETRY
   - ESTIMATED_CENTER
   - SUFFICIENT_GEOMETRY
7. 箱体从右侧进入并向左运动。
8. 检测到左侧短边和一条长边时：
   center_uv =
       corner
       + length_inward * L/2
       + width_inward * W/2
9. 只检测到一条长边时，
   不允许声称得到完整箱体中心。
10. 候选接触右边界且边线不足时，
    输出 PARTIAL_ENTERING。
11. 所有输出不得设置
    pose_valid_for_grasp = true。
```

---

## 55. 给 Codex 的第五轮任务

```text
实现 MotionEstimator 和 DetectionService。

要求：

1. 运动参考优先使用箱体左侧前沿。
2. 禁止使用可见点云质心作为首选运动参考。
3. 对短窗口内前沿位置拟合：
   s(t) = velocity * t + offset
4. 输出：
   - velocity_mps
   - position_span_m
   - fit_rmse_m
   - confidence
   - MotionState
5. DetectionService：
   - 收集短窗口
   - 构建运动掩膜
   - 提取候选
   - 向右扩张候选框
   - 生成相机坐标点云
   - 平面拟合
   - 边线拟合
   - 凹槽验证
   - 部分可见定位
   - 运动估计
   - 输出相机坐标观察位姿
   - 通过 T_base_camera 转换到 base
6. 根据 MotionState 和 GeometryState
   组合 DetectionStatus。
7. 生成中文 message。
8. 所有状态必须：
   pose_valid_for_grasp = false。
```

---

## 56. 第一阶段验收标准

1. 工程使用 C++17；
2. 不使用 GoogleTest；
3. 不建立传送带坐标系；
4. 检测和定位在相机坐标系中完成；
5. 最后通过 `T_base_camera` 转换到 `base`；
6. 不使用长期固定深度背景作为主方案；
7. 使用短窗口间隔帧深度差分；
8. 机器人站位小幅变化时仍能生成运动候选；
9. 能识别箱体从右侧进入、向左运动；
10. 只看到左侧一小部分时返回 `PARTIAL_ENTERING`；
11. 不把可见点云质心当作完整中心；
12. 运动估计优先跟踪左侧前沿；
13. 使用三维点估计上沿平面；
14. 边线在局部米制二维平面中使用 RANSAC；
15. 二维点不是图像像素；
16. 能进行部分或完整凹槽验证；
17. 检测到左侧短边和一条长边时，可用已知尺寸推算观察中心；
18. 只检测到一条长边时，不声称得到完整中心；
19. 输出相机坐标和 base 坐标；
20. 保存日志和调试图；
21. 支持真实深度序列离线回放；
22. 所有输出均满足：

```cpp
result.pose_valid_for_grasp = false;
```

---

## 57. 第一阶段最终结论

当前系统需要明确区分：

```text
运动候选
部分箱体几何
推算观察中心
较完整观察位姿
最终抓取位姿
```

在传送带运动期间：

```text
可以检测运动箱体
可以提示箱体正在从右侧进入
可以输出部分几何状态
可以根据已知尺寸推算观察中心
可以转换到机器人 base 坐标
可以记录前沿运动和速度
```

但是：

```text
不能把运动中的观察位姿作为最终抓取位姿
不能预测人工停止后的最终位置
不能因为检测到中心就允许机器人抓取
```

第一阶段始终：

```cpp
result.pose_valid_for_grasp = false;
```

下一阶段再实现：

```text
人工停止传送带
    ↓
视觉确认箱体稳定静止
    ↓
重新采集静态深度图
    ↓
静态多帧融合
    ↓
高精度开口箱定位
    ↓
生成真正可用于抓取的 grasp_pose
```
