# 点云过滤与固定尺寸箱体位置优化方案

本文说明新方案中两个连续的算法环节：

~~~
YOLOv8-World bbox
    -> 对齐深度 ROI
    -> 相机坐标系点云
    -> 点云过滤、候选箱体聚类、传送带平面 RANSAC
    -> 固定姿态、固定尺寸箱体位置优化
    -> 箱体中心位置
~~~

本文只讨论箱体位置估计，不估计箱体的 6D 姿态。箱体姿态被视为已知约束，最终服务接口仍然返回原来的 geometry_msgs/Pose。

## 1. 已知条件与边界

### 1.1 箱体尺寸

箱体尺寸固定，并且在 base_link 中约定为：

~~~
X方向长度：0.295 m
Y方向长度：0.395 m
Z方向高度：0.225 m
~~~

对应半尺寸：

~~~
h_x = 0.1475 m
h_y = 0.1975 m
h_z = 0.1125 m
~~~

箱体几何中心记为：

~~~
C = [C_x, C_y, C_z]^T
~~~

优化时只允许 C 平移，箱体的 roll、pitch、yaw 不作为优化变量。

### 1.2 坐标系和外参

原外参含义保持不变：

~~~
p_base = T_base_camera * p_camera
~~~

记：

~~~
T_base_camera = [ R_bc  t_bc ]
                [  0     1  ]
~~~

其中 R_bc 将相机坐标系中的向量转换到 base_link。

箱体姿态固定且与 base_link 三轴对齐，因此：

~~~
R_base_box = I
R_camera_box = R_bc^T
~~~

点云过滤和位置优化全部在 camera 中进行，最后只将中心点转换到 base_link：

~~~
点云和模型优化：camera
输出位置：base_link
~~~

### 1.3 深度数据来源

实时数据由 RealSense SDK 获取：

1. 获取彩色帧和深度帧。
2. 使用 rs.align(rs.stream.color) 将深度对齐到彩色图。
3. YOLO bbox 和深度像素使用同一图像坐标系。
4. 使用对齐后的深度内参进行反投影。

当前相机模块已经完成这部分工作。

离线测试目录中的 .raw 深度虽然是 640 x 480、Z16，但 metadata 中的深度焦距约为 385，与实时对齐深度使用的彩色坐标内参约 606 不同。因此当前 .raw 更像原始深度坐标，不能直接使用彩色图 YOLO bbox 对它做像素裁剪，除非同时提供彩色和深度外参并离线完成对齐。

## 2. 总体数据流

~~~mermaid
flowchart TD
    A[detect request] --> B[SDK aligned RGB-D]
    B --> C[YOLO crate bbox]
    C --> D[expand and clamp bbox]
    B --> D
    D --> E[depth range filtering]
    E --> F[back projection to camera cloud]
    F --> G[statistical outlier removal]
    G --> H[3D candidate clustering]
    H --> I[select or merge crate clusters]
    I --> J[conveyor support plane RANSAC]
    J --> K[remove support plane inliers]
    K --> L[fixed orientation and size optimization]
    L --> M[quality gate]
    M --> N[camera center]
    N --> O[T base camera]
    O --> P[base link position]
~~~

各步骤职责：

| 环节 | 主要目的 | 是否改变箱体模型 |
|---|---|---|
| 深度范围过滤 | 去除无效深度和明显远近异常 | 否 |
| 统计离群点过滤 | 去除孤立噪声点 | 否 |
| 候选深度聚类 | 从 bbox 内背景和箱体点中分离箱体候选 | 否 |
| 支撑平面 RANSAC | 找到传送带平面并移除平面点 | 否 |
| 固定尺寸优化 | 用箱体尺寸先验补全不可见部分并估计中心 | 是，使用固定模型 |
| 外参转换 | 将中心从 camera 转到 base_link | 否 |

## 3. 输入数据和 ROI 构造

### 3.1 YOLO 输出

YOLO 输出：

~~~
class_name
confidence
u_min, v_min, u_max, v_max
~~~

YOLO bbox 只表示目标在图像中的大致范围，不直接代表箱体几何中心，也不直接代表箱体完整投影范围。

### 3.2 bbox 扩张

对 bbox 增加小的像素边界：

~~~
u0 = max(0, u_min - margin)
v0 = max(0, v_min - margin)
u1 = min(image_width,  u_max + margin)
v1 = min(image_height, v_max + margin)
~~~

扩张的目的：

- 保留箱体边缘深度点。
- 避免 YOLO 框略微偏小导致箱体侧面被切掉。
- 给传送带平面 RANSAC 留出一部分箱体周围区域。

扩张不能过大。扩张过大后，办公环境、墙面、地面和其他物体会进入 ROI，增加聚类和模型拟合的歧义。

当前默认值：

~~~yaml
depth_bbox_margin_px: 4
~~~

### 3.3 深度范围过滤

对 ROI 中每个深度值 D(u,v) 进行转换：

~~~
Z(u,v) = D(u,v) * depth_scale
~~~

保留条件：

~~~
0 < D(u,v)
depth_min_m <= Z(u,v) <= depth_max_m
~~~

当前默认范围：

~~~yaml
depth_min_m: 0.30
depth_max_m: 3.00
~~~

该步骤只删除明显无效的点，不负责区分箱体和背景。

## 4. 深度反投影为相机坐标系点云

### 4.1 针孔反投影

对齐深度图中的像素 (u,v) 使用对齐后的彩色内参：

~~~
fx, fy, cx, cy
~~~

反投影公式：

~~~
X = (u - cx) * Z / fx
Y = (v - cy) * Z / fy
Z = Z
~~~

得到相机坐标系点：

~~~
p_i_camera = [X_i, Y_i, Z_i]^T
~~~

RealSense 相机坐标约定为：

~~~
X：图像向右
Y：图像向下
Z：相机朝向前方
~~~

### 4.2 像素采样步长

不必使用 ROI 中的每一个像素，可以使用固定步长采样：

~~~yaml
pointcloud_pixel_stride: 2
~~~

步长为 2 时，每隔一个像素取一个深度点，减少计算量，同时保留足够的空间密度。

步长过大可能造成箱体表面点不足、RANSAC 内点减少、聚类边界不稳定。步长过小则会增加计算量和深度噪声的重复计算。

### 4.3 点云数据结构

每个有效点至少保存：

~~~
point_camera_m: [X, Y, Z]
pixel_uv:       [u, v]
depth_m:        Z
~~~

保留像素坐标的原因：

- 可以回到图像检查点云来源。
- 可以计算每个聚类覆盖的图像区域。
- 可以检查聚类是否只来自 bbox 的一小块。

## 5. 统计离群点过滤

### 5.1 为什么需要过滤

D435 深度常见问题包括：

- 反光或黑色区域产生空洞。
- 箱体边缘出现飞点。
- 深度边界混合前景和背景。
- 某些像素深度跳到远处背景。
- 传送带纹理产生少量孤立点。

这些点如果直接进入平面 RANSAC 或箱体优化，可能造成聚类错误连接、箱体尺寸被拉大、箱体中心向背景方向偏移。

### 5.2 SOR 算法

对点云中每个点 p_i 找到 k 个近邻，计算平均近邻距离：

~~~
d_i = mean_j ||p_i - p_ij||_2
~~~

对所有点的 d_i 计算均值和标准差：

~~~
mu    = mean(d_i)
sigma = std(d_i)
~~~

保留条件：

~~~
d_i <= mu + alpha * sigma
~~~

其中 k 是近邻数量，alpha 是标准差倍数。

当前配置预留：

~~~yaml
pointcloud_statistical_neighbors: 20
pointcloud_statistical_std_ratio: 1.5
~~~

### 5.3 SOR 的限制

SOR 不是目标分割算法，只能删除空间上孤立的点。它无法单独解决连续的背景墙面、传送带平面、箱体内部连续点或箱体与背景相连的问题。因此 SOR 后仍然必须进行候选聚类和支撑平面处理。

## 6. 候选箱体深度聚类

### 6.1 聚类目标

YOLO bbox 内通常包含：

~~~
箱体外壁
箱体内壁或箱底
传送带
箱体后方背景
箱体边缘噪声
~~~

聚类的目的不是得到最终箱体模型，而是先回答：

~~~
哪些 3D 点更可能属于当前检测到的箱体？
~~~

### 6.2 推荐的 3D 聚类方式

在 SOR 后的相机系点云上进行欧氏距离聚类或 DBSCAN：

~~~
邻域条件：||p_i - p_j||_2 <= eps
最小点数：min_cluster_points
~~~

聚类使用 3D 距离，而不是只使用深度值，原因是同一深度的不同物体不能仅靠 Z 区分，而箱体前壁、侧壁和上沿可能具有不同深度。

~~~mermaid
flowchart TD
    A[ROI camera cloud] --> B[SOR filtered cloud]
    B --> C[voxel downsample optional]
    C --> D[3D Euclidean or DBSCAN clustering]
    D --> E[cluster statistics]
    E --> F[score clusters]
    F --> G[select main crate cluster]
    G --> H[merge compatible nearby clusters]
~~~

### 6.3 聚类统计量

对每个聚类 G_j 计算：

~~~
N_j              点数
centroid_j       质心
z_near_j         最近深度
z_median_j       深度中位数
z_far_j          最远深度
bbox_pixel_j     像素覆盖范围
extent_camera_j  3D 包围盒尺寸
~~~

像素覆盖范围用于检查聚类是否只占 YOLO bbox 的一个小角落。

### 6.4 聚类选择不能只看最近点

不建议直接选择 z_near 最小的聚类作为箱体，因为箱体前沿噪声可能形成最近的小聚类，近处也可能存在机械臂、桌边或其他遮挡物。

推荐使用组合评分：

~~~
score_j =
    w_count    * normalized(N_j)
  + w_coverage * image_coverage_j
  + w_depth    * foreground_consistency_j
  + w_size     * size_plausibility_j
~~~

各项含义：

- normalized(N_j)：点数越多，连续观测越可信。
- image_coverage_j：聚类在 YOLO bbox 中覆盖越充分越可信。
- foreground_consistency_j：深度分布应属于前景目标，而不是远处背景。
- size_plausibility_j：粗略投影到箱体方向后，尺寸不能明显小于噪声尺度，也不能远超已知箱体尺寸。

### 6.5 聚类合并

开口箱体的前壁、后壁、左右侧壁可能因为遮挡或深度间断形成多个聚类。若两个聚类满足以下条件，可以合并：

~~~
3D 最近距离 < merge_distance
投影 bbox 有重叠或相邻
深度范围与同一箱体模型兼容
合并后尺寸不超过模型尺寸加容差
~~~

合并的目的是将一个物理箱体的多个可见表面重新组成一个候选集合，不能无条件合并所有临近聚类，否则传送带和背景会重新进入箱体候选。

## 7. 传送带支撑平面 RANSAC

### 7.1 平面模型

传送带在当前阶段被视为静止、局部近似平面。平面方程为：

~~~
aX + bY + cZ + d = 0
~~~

写成向量形式：

~~~
n^T p + d = 0
~~~

其中 n=[a,b,c]^T 且 ||n||=1。

### 7.2 RANSAC 拟合流程

每次从点云中随机取三个不共线点，生成候选平面：

~~~
n_candidate = normalize((p2 - p1) cross (p3 - p1))
d_candidate = -n_candidate^T p1
~~~

计算所有点到候选平面的距离：

~~~
distance_i = |n_candidate^T p_i + d_candidate|
~~~

若：

~~~
distance_i <= plane_distance_threshold
~~~

则认为点 p_i 是该平面的内点。重复多次，选择内点数最多且满足方向约束的候选平面，最后使用所有内点重新拟合平面。

### 7.3 平面方向约束

仅靠 RANSAC 可能把箱体侧壁、桌面或背景墙识别成支撑平面，因此使用已知机器人姿态对法向量加约束。

在 base_link 中，传送带法向量近似为：

~~~
n_base_expected = [0, 0, 1]^T
~~~

转换到相机坐标系：

~~~
n_camera_expected = R_bc^T * n_base_expected
~~~

候选平面的方向约束：

~~~
angle(n_candidate, n_camera_expected) <= theta_max
~~~

因为法向量正负方向等价，实际比较使用：

~~~
angle = acos(|n_candidate^T n_camera_expected|)
~~~

当前配置预留：

~~~yaml
support_plane_max_normal_angle_deg: 12.0
~~~

### 7.4 支撑平面的选择

如果 ROI 中存在多个近似水平平面，应综合判断：

1. 平面内点比例足够高。
2. 平面法向量满足方向约束。
3. 平面位于箱体候选点云下方。
4. 箱体候选点云到平面存在接近 0.225 m 的高度范围。
5. 平面不是箱体上沿或箱底内部的局部小平面。

推荐评分：

~~~
plane_score =
    w_inlier * inlier_ratio
  + w_normal * normal_alignment
  + w_support * support_relation
~~~

不应只选择内点数最多的平面，因为背景地面可能比箱体下方传送带拥有更多点。

### 7.5 移除支撑平面

确定支撑平面后，删除：

~~~
|n^T p_i + d| <= plane_distance_threshold
~~~

的平面内点。

但是平面模型本身不能丢弃。后续箱体位置优化需要使用它约束箱体底面高度。

因此平面处理的输出是：

~~~
filtered_box_points
support_plane_model = (n, d)
support_plane_quality
~~~

### 7.6 找不到支撑平面时

可能原因：

- 箱体占满整个 bbox，周围没有传送带露出。
- 传送带被箱体完全遮挡。
- 深度缺失严重。
- 传送带表面反光。
- 外参方向或传送带姿态与配置不一致。

可选策略：

~~~
策略 A：保留点云，但 support_plane_valid=false，降低结果置信度
策略 B：使用点云范围估计 Z，但只允许观察结果，不允许抓取
策略 C：直接返回检测失败，等待下一次服务请求
~~~

第一版建议使用策略 C 或低置信度失败，不使用虚假的高度约束。

## 8. 过滤阶段总流程

~~~mermaid
flowchart TD
    A[aligned depth ROI] --> B[depth scale conversion]
    B --> C[range filter]
    C --> D{enough valid points}
    D -- no --> X[point cloud failure]
    D -- yes --> E[back projection]
    E --> F[statistical outlier removal]
    F --> G[3D clustering]
    G --> H{crate cluster valid}
    H -- no --> Y[cluster failure]
    H -- yes --> I[select and merge compatible clusters]
    I --> J[plane RANSAC]
    J --> K{support plane valid}
    K -- no --> Z[low confidence or failure]
    K -- yes --> L[remove plane inliers and keep plane model]
    L --> M[optimization input]
~~~

## 9. 固定尺寸箱体位置优化

### 9.1 优化输入

优化器输入：

~~~
P = {p_i_camera}       过滤后的箱体候选点
bbox                  YOLO 图像框
K = (fx, fy, cx, cy)  对齐彩色内参
plane = (n, d)        传送带支撑平面，可选但推荐有效
T_base_camera         原外参
Lx, Ly, Lz            已知箱体尺寸
~~~

优化输出：

~~~
C_camera = [Cx, Cy, Cz]^T
C_base   = R_bc * C_camera + t_bc
~~~

### 9.2 箱体模型

在箱体局部坐标系中，箱体范围为：

~~~
-h_x <= q_x <= h_x
-h_y <= q_y <= h_y
-h_z <= q_z <= h_z
~~~

相机点 p_i_camera 转换到候选箱体局部坐标：

~~~
q_i = R_camera_box^T * (p_i_camera - C_camera)
~~~

由于 R_camera_box=R_bc^T，所以：

~~~
q_i = R_bc * (p_i_camera - C_camera)
~~~

这里的 q_i 直接使用与 base_link 一致的箱体轴向。

### 9.3 为什么不是直接取点云质心

点云质心不能直接作为箱体中心：

- 相机只能看到箱体部分表面。
- 前壁、后壁、侧壁的可见比例不同。
- 箱体内部可能有深度点，但这些点不均匀分布。
- bbox 内包含传送带和背景。
- 箱体部分出画面时，质心会向可见区域偏移。

已知尺寸优化的作用，是寻找一个固定大小的虚拟箱体，使其位置能够解释可见点云，并用尺寸和支撑平面补全不可见部分。

### 9.4 优化变量

只有三个平移变量：

~~~
x = [Cx, Cy, Cz]^T
~~~

不优化：

~~~
箱体长度、宽度、高度
roll、pitch、yaw
箱体形状
~~~

这样可以避免在只有部分点云时同时估计 6D 姿态造成的多解问题。

## 10. 优化目标函数

推荐使用带鲁棒损失的组合目标：

~~~
J(C) =
    w_contain * J_contain(C)
  + w_surface * J_surface(C)
  + w_support * J_support(C)
  + w_image * J_image(C)
~~~

### 10.1 箱体包含约束

对每个点计算局部坐标 q_i，箱体外部距离向量为：

~~~
e_out_i = max(|q_i| - h, 0)
~~~

其中 max 按分量计算，h=[h_x,h_y,h_z]。

点在箱体内部或表面时 e_out_i 为零；点在箱体外部时产生超出箱体范围的惩罚：

~~~
r_contain_i = ||e_out_i||_2
J_contain(C) = sum_i rho(r_contain_i^2)
~~~

rho 是鲁棒损失函数，例如 soft-L1 或 Huber。该项防止优化后的虚拟箱体无法包含观测到的箱体点。

### 10.2 表面贴合约束

仅使用包含约束可能让箱体移动到较大范围，只要仍然包含点云即可。因此还需要让靠近箱体外表面的点贴近模型表面。

对局部点 q_i，近似计算到最近面的距离：

~~~
r_surface_i = min(
    |h_x - |q_x||,
    |h_y - |q_y||,
    |h_z - |q_z||
)
~~~

实际实现时应对不同点使用不同权重：

- 靠近外壁、底面、上沿的点：表面残差权重高。
- 位于箱体内部的点：优先使用包含约束，表面残差权重低。
- 明显背景点：由聚类剔除或用鲁棒损失降权。

组合为：

~~~
J_surface(C) = sum_i w_surface_i * rho(r_surface_i^2)
~~~

因为目标是开口箱体，不能把所有内部深度点都强行当作外表面点，否则箱底或箱内物体会把箱体中心拉偏。

### 10.3 传送带支撑约束

设箱体 Z 轴在相机坐标系中的单位向量为：

~~~
a_z = R_camera_box * [0, 0, 1]^T
~~~

箱体底面中心为：

~~~
p_bottom = C_camera - h_z * a_z
~~~

支撑平面为：

~~~
n^T p + d = 0
~~~

要求箱体底面中心位于支撑平面上：

~~~
r_support(C) = n^T p_bottom + d
J_support(C) = rho(r_support(C)^2)
~~~

该项对 C_z 特别重要，可以减少深度噪声造成的高度抖动。如果支撑平面无效，则不应使用这一项，或者将结果标记为低置信度。

### 10.4 图像投影约束

将固定尺寸箱体的 8 个角点投影到图像：

~~~
u = fx * X / Z + cx
v = fy * Y / Z + cy
~~~

得到模型投影框：

~~~
bbox_model = [u_min_model, v_min_model,
              u_max_model, v_max_model]
~~~

与 YOLO bbox 比较：

~~~
r_image = bbox_model - bbox_yolo
~~~

这个项只能作为弱约束，因为 YOLO 框可能只覆盖箱体可见部分，且箱体可能被图像边界截断。

如果 YOLO bbox 接触图像边界，对应方向不能使用严格等式约束，应改成不等式约束。例如右边界被截断时，只要求模型右边界不小于图像右边界，而不是强制相等。

### 10.5 总目标

完整目标可写成：

~~~
J(C) =
    w_contain * sum_i rho(r_contain_i^2)
  + w_surface * sum_i w_i * rho(r_surface_i^2)
  + w_support * rho(r_support^2)
  + w_image * rho(||r_image||^2)
~~~

当前 YAML 中的权重预留为：

~~~yaml
optimizer_surface_weight: 1.0
optimizer_containment_weight: 2.0
optimizer_support_weight: 4.0
optimizer_bbox_weight: 0.25
optimizer_robust_loss_scale_m: 0.020
~~~

这些权重不是物理量，必须通过现场数据调试。图像 bbox 项不能过强，否则部分可见箱体会被错误拉向图像框中心。

## 11. 优化初值与搜索

### 11.1 初始深度

可以使用候选点云的深度中位数或近端分位数得到初始观察距离：

~~~
z_init = percentile({Z_i}, 0.35 ~ 0.50)
~~~

不能使用单个最近点作为初值，因为最近点可能是边缘飞点。

### 11.2 支撑平面初值

若支撑平面有效，则由箱体底面接触关系约束中心位置：

~~~
n^T (C_camera - h_z * a_z) + d = 0
~~~

该关系可以直接给出中心沿平面法向量方向的初始位置。

### 11.3 X/Y 初值

可以使用以下信息生成候选初值：

- 候选点云质心沿箱体轴方向的位置。
- 可见点云在箱体轴方向的 min/max 与半尺寸组合。
- YOLO bbox 中心像素对应的相机射线。
- 固定尺寸模型投影框与 YOLO 框的粗匹配。

推荐先生成少量候选平移，再从最优候选进行局部优化，而不是完全依赖单个质心初值。

### 11.4 有界鲁棒优化

优化器可以使用 scipy.optimize.least_squares 或等价的有界非线性最小二乘方法：

~~~
变量：C_camera = [Cx, Cy, Cz]
损失：soft-L1 或 Huber
边界：工作空间 ROI
最大迭代：optimizer_max_iterations
~~~

当前预留：

~~~yaml
optimizer_max_iterations: 200
~~~

边界用于防止优化跳到相机后方、把背景平面解释成箱体，或把箱体中心放到机器人不可达区域之外。

## 12. 优化流程图

~~~mermaid
flowchart TD
    A[filtered crate candidate points] --> B[read fixed dimensions]
    B --> C[derive box axes in camera]
    C --> D[initialize center from depth and support plane]
    D --> E[transform points to box local coordinates]
    E --> F[containment residual]
    E --> G[surface residual]
    D --> H[support plane residual]
    D --> I[project model corners]
    I --> J[weak image bbox residual]
    F --> K[robust weighted objective]
    G --> K
    H --> K
    J --> K
    K --> L[bounded translation optimization]
    L --> M{quality gate}
    M -- fail --> N[return detection failure]
    M -- pass --> O[C camera]
    O --> P[T base camera]
    P --> Q[C base]
~~~

## 13. 质量检查和失败条件

优化完成后不能只检查优化器是否返回数值，还要检查：

~~~
1. 有效点数 >= pointcloud_min_points
2. 聚类点数和覆盖率达标
3. SOR 后点云没有全部消失
4. 支撑平面内点比例达标，或明确标记为无支撑平面
5. 箱体模型外部超出点比例不能过高
6. 表面残差 RMSE <= optimizer_max_surface_rmse_m
7. 有效点内点比例 >= optimizer_min_surface_inlier_ratio
8. 投影模型与 YOLO 结果有合理重叠
9. C_base 位于 output_base_* 范围
10. C_camera 位于相机前方且深度为正
~~~

失败时：

~~~
success = false
box_pose = fallback_pose
message = 具体失败阶段和原因
~~~

不能因为 YOLO 检测成功就直接把 bbox 中心或点云质心作为最终箱体中心。

## 14. 箱体部分可见时的行为

### 14.1 只看到箱体前部

如果前壁和传送带平面可见：

- 前壁点约束箱体距离。
- 支撑平面约束箱体高度。
- 固定尺寸补全后方不可见长度。

此时可以估计中心，但置信度低于完整可见场景。

### 14.2 只看到箱体上沿或少量局部

如果只有很少的上沿点：

- 点云可能无法区分箱体前后方向。
- 固定尺寸模型可能有多个平移解。
- 图像 bbox 也可能只是局部框。

此时应通过质量检查拒绝结果，不能强行输出一个看似合理的中心。

### 14.3 箱体部分出画面

如果 bbox 接触图像边界：

- 接触边界方向的模型投影只能使用不等式约束。
- 不能要求完整模型框与 YOLO 框完全相等。
- 如果剩余可见尺寸不足以约束隐藏方向，应返回低置信度或失败。

### 14.4 箱体完全不在视野内

YOLO 没有 bbox 时，后续所有点云和优化环节都不执行，服务直接返回无箱体失败消息。

## 15. 与服务接口的关系

服务接口不变：

~~~
service: /detect
type: upper_limb_interface/srv/DetectAprilTag
~~~

服务内部状态为：

~~~
capture RGB-D
    -> YOLO bbox
    -> point cloud
    -> filtered candidate
    -> support plane
    -> fixed-size optimization
    -> output validation
~~~

geometry_msgs/Pose 中的位置字段为最终箱体中心：

~~~
box_pose.position = C_base
~~~

由于当前不估计箱体旋转，方向字段使用单位四元数：

~~~
qx = 0
qy = 0
qz = 0
qw = 1
~~~

四元数不是点云优化变量，只是为了保持原服务消息结构完整。

## 16. 参数建议

当前相关 YAML 参数：

~~~yaml
depth_min_m: 0.30
depth_max_m: 3.00
depth_bbox_margin_px: 4
pointcloud_pixel_stride: 2
pointcloud_min_points: 300
pointcloud_statistical_neighbors: 20
pointcloud_statistical_std_ratio: 1.5

support_plane_distance_threshold_m: 0.012
support_plane_ransac_iterations: 300
support_plane_min_inlier_ratio: 0.10
support_plane_max_normal_angle_deg: 12.0

optimizer_surface_weight: 1.0
optimizer_containment_weight: 2.0
optimizer_support_weight: 4.0
optimizer_bbox_weight: 0.25
optimizer_robust_loss_scale_m: 0.020
optimizer_max_iterations: 200
optimizer_max_surface_rmse_m: 0.035
optimizer_min_surface_inlier_ratio: 0.35
~~~

参数调试顺序建议：

1. 先确认深度单位和 ROI 点数。
2. 再调 depth_min_m/depth_max_m。
3. 再调 SOR 的邻居数和标准差倍数。
4. 再调聚类距离和最小点数。
5. 再调支撑平面距离、法向角度和最小内点比例。
6. 最后调优化目标权重和质量阈值。

不建议在点云仍包含大量背景时直接调优化权重，因为此时优化器看到的输入已经不符合箱体模型。

## 17. 当前实现状态

截至本文创建时：

已实现：

~~~
RealSense SDK RGB-D 获取
深度对齐到彩色图
YOLOv8-World 箱体检测
bbox 扩张与边界裁剪
深度范围过滤
相机系点云反投影
最小有效点数检查
~~~

待实现：

~~~
统计离群点过滤
3D 候选箱体聚类与合并
传送带支撑平面 RANSAC
箱体固定尺寸优化
位置质量检查
camera 到 base_link 的最终位置输出
~~~

因此当前服务返回的点云信息只证明数据已经准备完成，尚不能作为机械臂抓取位置。

## 18. 下一步实现顺序

建议按以下顺序继续：

~~~
1. 在当前 CameraPointCloud 上实现 SOR
2. 实现 3D 欧氏聚类和候选评分
3. 实现支撑平面 RANSAC，并保留 plane model
4. 将过滤后的点云和 plane model 接入优化器
5. 实现固定尺寸箱体的 C_camera 优化
6. 转换到 C_base 并通过原 /detect 返回
7. 使用真机重复请求和两组图像做误差评估
~~~

