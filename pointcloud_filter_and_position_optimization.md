# 点云过滤与固定尺寸箱体位置优化方案

本文对应当前 detect_pkg 的实际 Python ROS 2 实现，说明从 YOLOE-26 实例 mask 到箱体中心输出之间的点云处理、结构特征提取和固定尺寸位置优化流程。

当前方案只估计箱体中心位置，不估计箱体姿态。箱体尺寸固定，姿态只存在小幅扰动。当前不使用运动差分、传送带支撑平面或 AprilTag。

## 1. 当前总体链路

~~~text
/detect 服务请求
  -> RealSense SDK 获取并对齐 RGB-D
  -> YOLOE-26 根据文本提示分割箱体实例
  -> 选择置信度最高的有效 bbox + mask
  -> mask 边缘腐蚀与深度范围过滤
  -> 深度反投影为 camera 点云
  -> 统计离群点过滤 SOR
  -> 3D 欧氏连通聚类
  -> 选择箱体候选簇
  -> 远侧内壁平面 RANSAC
  -> 箱体真实上边沿线 RANSAC
  -> 固定尺寸模型的 3DoF 中心优化
  -> 上边沿高度锚定
  -> 完整候选点云质量检查
  -> camera 中心转换到 base_link
  -> 返回原 /detect 服务响应
~~~
当前不执行：

- 连续帧运动差分；
- 传送带支撑平面拟合和移除；
- 箱体 roll、pitch、yaw 优化；
- 完整箱体模型点云生成；
- depth_u16.png 保存。

### 1.1 总体流程图

~~~mermaid
flowchart TD
    A[Detect service request<br/>检测服务请求] --> B[RealSense aligned RGB-D<br/>获取并对齐 RGB-D]
    B --> C[YOLOE-26 segmentation<br/>文本提示实例分割]
    C --> D[Validate and erode mask<br/>校验并内缩 mask]
    B --> D
    D --> E[Mask and depth filtering<br/>mask 与深度联合过滤]
    E --> F[Back-project to camera cloud<br/>反投影到 camera 点云]
    F --> G[SOR outlier removal<br/>统计离群点过滤]
    G --> H[3D connected clustering<br/>三维连通聚类]
    H --> I[Select candidate cluster<br/>选择候选簇]
    I --> J[Far inner-wall RANSAC<br/>远侧内壁拟合]
    I --> K[Upper edge-line RANSAC<br/>真实上边沿拟合]
    J --> L[Fixed-size 3DoF optimization<br/>固定尺寸中心优化]
    K --> L
    I --> L
    L --> M[Upper-edge height anchoring<br/>上边沿高度锚定]
    M --> N[Quality gates on full cloud<br/>完整点云质量检查]
    N --> O[Camera center<br/>camera 系中心]
    O --> P[T_base_camera<br/>相机到 base_link 外参]
    P --> Q[Detect response<br/>返回服务响应]
~~~

## 2. 已知条件与坐标系

箱体外部尺寸固定为：

$$
L_x=0.295\,\mathrm{m},\qquad
L_y=0.395\,\mathrm{m},\qquad
L_z=0.225\,\mathrm{m}
$$

半尺寸为：

$$
h_x=0.1475\,\mathrm{m},\qquad
h_y=0.1975\,\mathrm{m},\qquad
h_z=0.1125\,\mathrm{m}
$$

唯一优化变量是箱体中心：

$$
\mathbf{C}_{camera}=[C_x,C_y,C_z]^{\mathsf T}
$$

RealSense camera 坐标约定：

~~~text
X：图像向右
Y：图像向下
Z：相机朝向前方
~~~

原外参语义保持不变：

$$
\mathbf{p}_{base}=
\mathbf{T}_{base\leftarrow camera}\mathbf{p}_{camera}
$$

令：

$$
\mathbf{T}_{base\leftarrow camera}=
\begin{bmatrix}
\mathbf{R}_{bc}&\mathbf{t}_{bc}\\
\mathbf{0}^{\mathsf T}&1
\end{bmatrix}
$$

箱体没有单独姿态标定，因此使用名义轴向：

$$
\mathbf{R}^{nominal}_{camera\leftarrow box}
=\mathbf{R}_{bc}^{\mathsf T}
$$

这不是箱体和 base_link 永远完全平行的物理断言。实际小姿态扰动可表示为：

$$
\mathbf{R}^{actual}_{base\leftarrow box}
=
\mathbf{R}^{nominal}_{base\leftarrow box}\Delta\mathbf{R}
$$

当前不优化 $\Delta\mathbf{R}$，通过结构特征方向门限、几何容差、鲁棒损失和质量门控处理。box_coordinate_z_sign 用于保持箱体 Z+ 开口方向约定。

## 3. RGB-D 与 ROI 点云

相机通过 pyrealsense2 SDK 获取彩色和深度流，并使用：

~~~python
rs.align(rs.stream.color)
~~~

将深度对齐到彩色图像。YOLOE bbox、整图 mask、深度像素和反投影内参处于同一对齐坐标系。

主要相机参数：

~~~yaml
camera_width: 640
camera_height: 480
camera_fps: 30
camera_keep_running: true
camera_warmup_frames: 30
camera_request_discard_frames: 5
~~~

首次请求可能包含相机启动、预热、首帧等待或硬件复位时间；相机保持运行后，后续请求只执行少量丢帧和取帧。

YOLOE-26 使用 `yoloe-26s-seg.pt` 和以下文本提示词：

~~~text
green plastic crate
plastic crate
storage crate
green storage box
green box
plastic container
~~~

每个候选实例包含检测框和与彩色图同尺寸的布尔 mask：

$$
\mathbf{b}_{yolo}=[u_{min},v_{min},u_{max},v_{max}]^{\mathsf T}
$$

$$
M(u,v)\in\{0,1\}
$$

bbox 只用于候选选择、限制数组读取范围、优化初值和弱投影约束；只有 mask 中的像素允许生成点云。

设 bbox 扩张像素为 m，图像宽高为 W、H：

$$
\begin{aligned}
u_0&=\max(0,u_{min}-m),&
v_0&=\max(0,v_{min}-m),\\
u_1&=\min(W,u_{max}+m),&
v_1&=\min(H,v_{max}+m)
\end{aligned}
$$

当前：

~~~yaml
depth_bbox_margin_px: 4
pointcloud_mask_erosion_px: 1
depth_min_m: 0.20
depth_max_m: 3.00
pointcloud_pixel_stride: 2
~~~

深度值 D(u,v) 通过比例因子 s_d 转为：

$$
Z(u,v)=D(u,v)s_d
$$

点云采样使用内缩后的 mask；原始完整实例 mask 仍用于调试保存。保留条件为：

$$
M_{eroded}(u,v)=1,\qquad D(u,v)>0,\qquad
0.20\leq Z(u,v)\leq 3.00
$$

对齐深度内参为 $(f_x,f_y,c_x,c_y)$ 时：

$$
X_i=\frac{(u_i-c_x)Z_i}{f_x},\qquad
Y_i=\frac{(v_i-c_y)Z_i}{f_y},\qquad
Z_i=Z_i
$$

得到 camera 点：

$$
\mathbf{p}_i^{camera}=
[X_i,Y_i,Z_i]^{\mathsf T}
$$

CameraPointCloud 同时保存三维点、像素坐标和深度值，像素坐标用于调试 PLY 的颜色映射。

## 4. 统计离群点过滤 SOR

SOR 用于删除 mask 点云中的孤立飞点和深度噪声；二维目标分割由 YOLOE-26 完成。

对每个点查询 k 个近邻，计算平均距离：

$$
d_i=
\frac{1}{k}\sum_{j\in\mathcal{N}_k(i)}
\|\mathbf{p}_i-\mathbf{p}_j\|_2
$$

再计算：

$$
\mu_d=\operatorname{mean}(d_i),\qquad
\sigma_d=\operatorname{std}(d_i)
$$

保留条件：

$$
d_i\leq\mu_d+\alpha\sigma_d
$$

当前：

~~~yaml
pointcloud_statistical_neighbors: 20
pointcloud_statistical_std_ratio: 1.5
~~~

## 5. 三维候选聚类

SOR 后使用 cKDTree 半径邻域和 BFS 建立三维连通分量：

~~~mermaid
flowchart TD
    A[SOR filtered points<br/>SOR 后点云] --> B[Build cKDTree<br/>建立空间索引]
    B --> C[Radius neighbors<br/>搜索半径邻域]
    C --> D[Connected component BFS<br/>连通分量遍历]
    D --> E[Remove small components<br/>删除小分量]
    E --> F[Score count coverage depth<br/>数量、覆盖、深度评分]
    F --> G[Select highest score<br/>选择最高分候选簇]
~~~

当前：

~~~yaml
pointcloud_cluster_tolerance_m: 0.025
pointcloud_min_cluster_points: 40
~~~

每个簇计算点数、相机系质心、图像范围、深度范围、图像覆盖率和深度一致性。评分为：

$$
s=0.65s_{count}+0.20s_{coverage}+0.15s_{depth}
$$

最高分簇作为 selected_cloud。当前不执行支撑平面过滤，也不额外合并多个簇。

## 6. 结构特征提取

### 6.1 远侧内壁平面

根据箱体名义轴向选择 inner_wall_axis，当前默认使用箱体局部 X 轴。先选择较远侧点：

~~~yaml
inner_wall_axis: x
inner_wall_far_quantile: 0.55
~~~

再用 RANSAC 拟合：

$$
\mathbf{n}^{\mathsf T}\mathbf{p}+d=0
$$

每轮从三个点估计平面，检查点到平面距离和法向夹角，最终用 SVD 重新拟合内点。

~~~yaml
inner_wall_distance_threshold_m: 0.015
inner_wall_ransac_iterations: 300
inner_wall_min_inlier_ratio: 0.05
inner_wall_max_normal_angle_deg: 15.0
~~~

实测法向用于验证；中心优化使用名义法向，避免把小姿态误差直接解释为中心偏移。

### 6.2 真实上边沿线

上边沿通过三维直线 RANSAC 拟合，不使用单个高分位点。

流程：

1. 按箱体局部 Z 坐标取上部候选点；
2. 分别尝试局部 X、Y 两个水平轴；
3. 随机选择两个点构造候选直线；
4. 计算点到直线距离并筛选内点；
5. 以连续支持跨度和内点数量评分；
6. 用线性拟合得到上边沿局部 Z 坐标。

~~~yaml
top_edge_candidate_quantile: 0.70
top_edge_distance_threshold_m: 0.012
top_edge_ransac_iterations: 300
top_edge_min_span_ratio: 0.35
top_edge_max_direction_angle_deg: 12.0
~~~

该上边沿是当前中心 Z 方向的重要锚点。

## 7. 固定尺寸 3DoF 中心优化

### 7.1 局部坐标转换

设 R_camera_from_box 为箱体局部坐标到 camera 坐标的旋转矩阵：

$$
\mathbf{q}_i=
\mathbf{R}_{camera\leftarrow box}^{\mathsf T}
\left(
\mathbf{p}_i^{camera}-\mathbf{C}_{camera}
\right)
$$

q_i 是点在名义箱体坐标系中的坐标。实现使用等价的行向量形式：

~~~python
q = (points - center_camera) @ rotation_camera_from_box
~~~

### 7.2 固定尺寸模型距离

半尺寸为 h=[h_x,h_y,h_z]^T。超出量：

$$
\mathbf{o}_i=
\max\left(|\mathbf{q}_i|-\mathbf{h},\mathbf{0}\right)
$$

点在模型外部时：

$$
d_i^{outside}=\|\mathbf{o}_i\|_2
$$

点在模型内部时：

$$
d_i^{inside}=\min_j(h_j-|q_{i,j}|)
$$

最终距离：

$$
d_i=
\begin{cases}
d_i^{outside},&\text{点在模型外部}\\
d_i^{inside},&\text{点在模型内部}
\end{cases}
$$

表面残差：

$$
r_{surface,i}=
\frac{\max(d_i-\tau_{surface},0)}
{s_{robust}}
$$

当前：

~~~yaml
optimizer_model_weight: 1.0
optimizer_model_surface_tolerance_m: 0.03
optimizer_robust_loss_scale_m: 0.020
~~~

这里使用完整固定尺寸模型进行匹配，但不会生成或保存完整模型点云。

### 7.3 bbox 弱约束

根据固定尺寸模型 8 个角点投影得到模型框 b_model。模型覆盖不足时：

$$
\mathbf{r}_{cover}=
\max\left(
\begin{bmatrix}
u_{model,l}-u_{obs,l}\\
v_{model,t}-v_{obs,t}\\
u_{obs,r}-u_{model,r}\\
v_{obs,b}-v_{model,b}
\end{bmatrix},
\mathbf{0}
\right)
$$

同时使用带图像边界可见性权重的弱框差异项。bbox 接触边界时，对应方向的严格匹配权重逐渐降低，允许模型延伸到图像外。

~~~yaml
optimizer_bbox_weight: 0.25
optimizer_bbox_clip_transition_px: 30.0
~~~

### 7.4 内壁与上边沿约束

内壁约束：

$$
r_{wall}=
\mathbf{n}_{wall}^{\mathsf T}\mathbf{C}_{camera}
+d_{wall}+h_{wall}
$$

上边沿约束：

$$
C_{box,z}^{local}
=
z_{top}-\frac{L_z}{2}
$$

实现沿箱体 Z 轴修正中心，不改变箱体轴方向：

$$
\mathbf{C}'_{camera}
=
\mathbf{C}_{camera}
+
\left(
C_{box,z}^{target}-C_{box,z}^{current}
\right)
\mathbf{a}_{z}^{camera}
$$

~~~yaml
optimizer_inner_wall_weight: 2.0
optimizer_top_edge_weight: 1.0
optimizer_top_edge_tolerance_m: 0.0
~~~

### 7.5 总目标和鲁棒损失

当前残差为：

$$
\mathbf{r}(\mathbf{C})=
\begin{bmatrix}
\mathbf{r}_{surface}\\
\mathbf{r}_{wall}\ \text{（可选）}\\
r_{top}\ \text{（可选）}\\
\mathbf{r}_{bbox}
\end{bmatrix}
$$

优化目标：

$$
\min_{\mathbf{C}_{camera}}
\sum_k\rho\left(r_k(\mathbf{C})^2\right)
$$

默认 soft-L1 函数：

$$
\rho(z)=2\left(\sqrt{1+z}-1\right)
$$

优化变量只有 C_camera，不使用支撑平面残差，不优化姿态。

## 8. 优化降采样、初值和边界

优化阶段使用确定性的均匀代表性采样：

~~~yaml
optimizer_representative_point_count: 4000
~~~

完整候选点云少于该数量时不采样，超过时均匀选点。需要区分：

~~~text
优化：最多 4000 个代表性点
最终质量评估：完整 selected_cloud
调试 PLY：完整 selected_cloud + 1 个中心点
~~~

初值是优化器开始迭代时的中心猜测：

$$
\mathbf{C}^{(0)}_{camera}
=
[C_x^{(0)},C_y^{(0)},C_z^{(0)}]^{\mathsf T}
$$

当前保留有限数量的中位数、深度分位数、内壁修正和上边沿修正初值：

~~~yaml
optimizer_initial_seed_count: 4
~~~

若内壁和上边沿有效，优先保留中位数、内壁修正、上边沿修正和一个备用初值。减少初值只减少重复优化次数，不改变模型或残差。

中心搜索边界根据完整候选点云范围和固定模型角点深度范围生成，避免中心落到相机后方或偏离候选点云过远。

## 9. 质量门控

成功输出至少需要满足：

1. YOLOE-26 bbox 与实例 mask 有效；
2. ROI 有效深度点数达到 pointcloud_min_points；
3. SOR 后存在满足最小点数的候选簇；
4. 优化器返回有限中心并成功收敛；
5. 箱体中心位于相机前方；
6. base_link 中心位于输出范围；
7. 完整候选点云 RMSE 不超过 optimizer_max_surface_rmse_m；
8. 完整候选点云内点比例不低于 optimizer_min_surface_inlier_ratio；
9. 固定尺寸模型投影与 YOLOE bbox 存在合理重叠。

内壁或上边沿拟合失败时，对应约束被跳过，流程仍可继续。最终质量不达标时返回失败和 fallback_pose。

## 10. camera 到 base_link

$$
\mathbf{C}_{base}
=
\mathbf{R}_{bc}\mathbf{C}_{camera}
+\mathbf{t}_{bc}
$$

Pose 位置字段使用 C_base。当前不估计姿态，因此方向使用单位四元数：

~~~text
qx = 0
qy = 0
qz = 0
qw = 1
~~~

服务接口保持：

~~~text
service: /detect
type: upper_limb_interface/srv/DetectAprilTag
~~~

## 11. 调试输出与性能

为降低 `/detect` 延迟，生产配置默认关闭；现场检查 mask 时可临时开启：

~~~yaml
debug_enabled: false
debug_output_dir: /home/user/liujun/detect_box_ws/debug_box_position
debug_max_snapshots: 10
~~~

每次请求最多保留最近 10 个目录，包含：

~~~text
color_with_yoloe_mask.png
mask_roi_cloud.ply
~~~

color_with_yoloe_mask.png 包含完整实例 mask 叠加、bbox、提示词类别和置信度；mask_roi_cloud.ply 是 mask 深度直接反投影的点云。为降低服务延迟，不再计算或保存单独的 mask 图片、原始彩色图和最终候选点云。

不再保存：

~~~text
depth_u16.png
completed_box_model_cloud.ply
~~~

日志会打印 capture、yoloe_segment、pointcloud_extract、pointcloud_process、optimizer、pipeline、debug_snapshot 和 total。首次请求可能包含相机启动、预热、硬件复位和 YOLOE CUDA 初始化；相机保持运行并完成模型 warm-up 后，后续请求明显更快。

## 12. 当前限制

1. 当前不处理传送带导致的箱体运动，单次请求使用一组对齐 RGB-D 完成估计。
2. mask 是否覆盖开口箱体的内壁取决于模型输出；现场调试时检查 color_with_yoloe_mask.png 和 mask_roi_cloud.ply。
3. bbox 被图像边界截断时，模型投影采用弱边界覆盖约束，不能把 YOLOE 框当成完整箱体框。
4. 箱体姿态偏差过大时，固定姿态模型可能产生中心偏差或质量门控失败。
5. 内壁和上边沿是辅助特征，拟合失败时流程仍可继续，但质量可能下降。
6. 红色中心点是 PLY 中的额外顶点，不是完整箱体模型表面；CloudCompare 中需要开启 RGB 显示并适当增大点大小。
7. /detect 服务串行执行，同时请求会返回已有检测请求正在运行。

## 13. 相关文件

~~~text
src/detect_pkg/detect_box_pipeline/detect_server_node.py
  ROS 2 服务、流程编排、计时、响应和调试保存调用

src/detect_pkg/detect_box_pipeline/realsense_camera.py
  RealSense SDK、RGB-D 对齐、相机预热和持续运行

src/detect_pkg/detect_box_pipeline/yoloe_segmenter.py
  YOLOE-26 模型、文本提示、实例 mask 和设备选择

src/detect_pkg/detect_box_pipeline/box_position_estimator.py
  点云反投影、SOR、聚类、结构特征和中心优化

src/detect_pkg/detect_box_pipeline/debug_snapshot.py
  调试图像、候选 PLY 和中心红点保存

src/detect_pkg/config/box_position_estimation.yaml
  启动参数、尺寸、外参、优化阈值和调试配置

src/detect_pkg/launch/box_position_estimation.launch.py
  ROS 2 launch 入口
~~~
