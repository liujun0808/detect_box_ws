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

本文只讨论箱体位置估计，不估计箱体的 6D 姿态。箱体姿态只作为不参与优化的名义模型方向；实际存在的小姿态变化作为模型误差处理。最终服务接口仍然返回原来的 geometry_msgs/Pose。

## 1. 已知条件与边界

### 1.1 箱体尺寸

箱体尺寸固定，并且在 base_link 中约定为：

~~~
X方向长度：0.295 m
Y方向长度：0.395 m
Z方向高度：0.225 m
~~~

对应半尺寸为：

$$
h_x = 0.1475\,\mathrm{m},\qquad
h_y = 0.1975\,\mathrm{m},\qquad
h_z = 0.1125\,\mathrm{m}
$$

箱体几何中心记为：

$$
\mathbf{C} = [C_x, C_y, C_z]^\mathsf{T}
$$

优化时只允许 C 平移，箱体的 roll、pitch、yaw 不作为优化变量。

### 1.2 坐标系和外参

原外参含义保持不变：

$$
\mathbf{p}_{base} = \mathbf{T}_{base\leftarrow camera}\,
\mathbf{p}_{camera}
$$

记：

$$
\mathbf{T}_{base\leftarrow camera} =
\begin{bmatrix}
\mathbf{R}_{bc} & \mathbf{t}_{bc}\\
\mathbf{0}^{\mathsf{T}} & 1
\end{bmatrix}
$$

其中 R_bc 将相机坐标系中的向量转换到 base_link。

箱体每次检测的位置会变化，姿态只发生小变化，且本方案不估计姿态。由于没有额外的箱体姿态标定，位置优化采用 `base_link` 三轴作为箱体的名义模型轴向：

$$
\mathbf{R}_{base\leftarrow box}^{nominal}=\mathbf{I},\qquad
\mathbf{R}_{camera\leftarrow box}^{nominal}=\mathbf{R}_{bc}^{\mathsf{T}}
$$

这里的单位矩阵是未标定时的**名义建模约定**，不是断言箱体实际轴向与 `base_link` 完全一致。实际箱体姿态可表示为：

$$
\mathbf{R}_{base\leftarrow box}^{actual}
=\mathbf{R}_{base\leftarrow box}^{nominal}\Delta\mathbf{R},
\qquad \Delta\mathbf{R}\text{ 只表示未建模的小姿态扰动}
$$

$\Delta\mathbf{R}$ 不进入本次优化，也不需要输出。若实际姿态变化超过小误差范围，固定尺寸位置模型的误差和质量门控失败率都会增加，此时应返回低置信度或失败，而不是隐式估计姿态。

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
    A[detect request<br/>检测请求] --> B[SDK aligned RGB-D<br/>SDK 获取并对齐 RGB-D]
    B --> C[YOLO crate bbox<br/>YOLO 箱体检测框]
    C --> D[expand and clamp bbox<br/>扩张并裁剪检测框]
    B --> D
    D --> E[depth range filtering<br/>深度范围过滤]
    E --> F[back projection to camera cloud<br/>反投影到相机点云]
    F --> G[statistical outlier removal<br/>统计离群点过滤]
    G --> H[3D candidate clustering<br/>三维候选聚类]
    H --> I[select or merge crate clusters<br/>选择或合并箱体聚类]
    I --> J[optional support plane RANSAC<br/>可选传送带平面 RANSAC]
    J --> K{support plane valid?<br/>支撑平面是否有效}
    K -- yes / 是 --> L[remove plane inliers and keep model<br/>移除平面内点并保留模型]
    K -- no / 否 --> M[keep cloud and skip plane input<br/>保留点云并跳过平面输入]
    L --> N[fixed orientation and size optimization<br/>固定姿态尺寸优化]
    M --> N
    N --> O[quality gate<br/>质量门控]
    O --> P[camera center<br/>相机系中心]
    P --> Q[T base camera<br/>相机到 base_link 外参]
    Q --> R[base link position<br/>base_link 系位置]
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

$$
\begin{aligned}
u_0 &= \max(0,u_{min}-m), & v_0 &= \max(0,v_{min}-m),\\
u_1 &= \min(W,u_{max}+m), & v_1 &= \min(H,v_{max}+m)
\end{aligned}
$$

其中 $m$ 是扩张像素数，$W,H$ 是图像宽高。

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

对 ROI 中每个深度值 $D(u,v)$ 进行转换：

$$
Z(u,v)=D(u,v)\,s_d
$$

其中 $s_d$ 是 RealSense 深度比例因子。

保留条件：

$$
D(u,v)>0,\qquad z_{min}\leq Z(u,v)\leq z_{max}
$$

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

$$
X=\frac{(u-c_x)Z}{f_x},\qquad
Y=\frac{(v-c_y)Z}{f_y},\qquad
Z=Z
$$

得到相机坐标系点：

$$
\mathbf{p}_{i}^{camera}=[X_i,Y_i,Z_i]^{\mathsf{T}}
$$

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

对点云中每个点 $\mathbf{p}_i$ 找到 $k$ 个近邻，计算平均近邻距离：

$$
d_i=\frac{1}{k}\sum_{j=1}^{k}\lVert\mathbf{p}_i-\mathbf{p}_{ij}\rVert_2
$$

对所有点的 $d_i$ 计算均值和标准差：

$$
\mu=\operatorname{mean}(d_i),\qquad
\sigma=\operatorname{std}(d_i)
$$

保留条件：

$$
d_i\leq\mu+\alpha\sigma
$$

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

邻域条件为 $\lVert\mathbf{p}_i-\mathbf{p}_j\rVert_2\leq\varepsilon$，并设置最小点数 `min_cluster_points`。

聚类使用 3D 距离，而不是只使用深度值，原因是同一深度的不同物体不能仅靠 Z 区分，而箱体前壁、侧壁和上沿可能具有不同深度。

~~~mermaid
flowchart TD
    A[ROI camera cloud<br/>ROI 相机点云] --> B[SOR filtered cloud<br/>SOR 过滤点云]
    B --> C[voxel downsample optional<br/>可选体素降采样]
    C --> D[3D Euclidean or DBSCAN clustering<br/>三维欧氏或 DBSCAN 聚类]
    D --> E[cluster statistics<br/>计算聚类统计量]
    E --> F[score clusters<br/>聚类评分]
    F --> G[select main crate cluster<br/>选择主箱体聚类]
    G --> H[merge compatible nearby clusters<br/>合并兼容的相邻聚类]
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

$$
s_j=w_{count}\,\hat{N}_j+w_{coverage}\,s_{coverage,j}
 +w_{depth}\,s_{depth,j}+w_{size}\,s_{size,j}
$$

其中 $\hat{N}_j$ 是归一化点数，其他三项分别表示图像覆盖、前景深度一致性和尺寸合理性。

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

$$
aX+bY+cZ+d=0
$$

写成向量形式：

$$
\mathbf{n}^{\mathsf{T}}\mathbf{p}+d=0,\qquad
\mathbf{n}=[a,b,c]^{\mathsf{T}},\qquad \lVert\mathbf{n}\rVert_2=1
$$

### 7.2 RANSAC 拟合流程

每次从点云中随机取三个不共线点，生成候选平面：

$$
\mathbf{n}_{candidate}=
\frac{(\mathbf{p}_2-\mathbf{p}_1)\times(\mathbf{p}_3-\mathbf{p}_1)}
{\lVert(\mathbf{p}_2-\mathbf{p}_1)\times(\mathbf{p}_3-\mathbf{p}_1)\rVert_2},
\qquad
d_{candidate}=-\mathbf{n}_{candidate}^{\mathsf{T}}\mathbf{p}_1
$$

计算所有点到候选平面的距离：

$$
\delta_i=\left|\mathbf{n}_{candidate}^{\mathsf{T}}\mathbf{p}_i+d_{candidate}\right|
$$

若：

$$
\delta_i\leq\tau_{plane}
$$

则认为点 $\mathbf{p}_i$ 是该平面的内点。重复多次，选择内点数最多且满足方向约束的候选平面，最后使用所有内点重新拟合平面。

### 7.3 平面方向约束

仅靠 RANSAC 可能把箱体侧壁、桌面或背景墙识别成支撑平面，因此使用已知机器人姿态对法向量加约束。

在 base_link 中，传送带法向量近似为：

$$
\mathbf{n}_{base}^{expected}=[0,0,1]^{\mathsf{T}}
$$

转换到相机坐标系：

$$
\mathbf{n}_{camera}^{expected}=\mathbf{R}_{bc}^{\mathsf{T}}
\mathbf{n}_{base}^{expected}
$$

候选平面的方向约束：

$$
\operatorname{angle}(\mathbf{n}_{candidate},
\mathbf{n}_{camera}^{expected})\leq\theta_{max}
$$

因为法向量正负方向等价，实际比较使用：

$$
\theta=\arccos\left(\left|\mathbf{n}_{candidate}^{\mathsf{T}}
\mathbf{n}_{camera}^{expected}\right|\right)
$$

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

$$
s_{plane}=w_{inlier}s_{inlier}+w_{normal}s_{normal}+w_{support}s_{support}
$$

不应只选择内点数最多的平面，因为背景地面可能比箱体下方传送带拥有更多点。

### 7.5 移除支撑平面

确定支撑平面后，删除满足下式的平面内点：

$$
\left|\mathbf{n}^{\mathsf{T}}\mathbf{p}_i+d\right|
\leq\tau_{plane}
$$

但是平面模型本身不能丢弃。后续箱体位置优化需要使用它约束箱体底面高度。

因此平面处理的输出是：

~~~
filtered_box_points
support_plane_quality
~~~

有效时，支撑平面模型记为 $(\mathbf{n},d)$；无效时模型值为 `None`。

### 7.6 找不到支撑平面时

可能原因：

- 箱体占满整个 bbox，周围没有传送带露出。
- 传送带被箱体完全遮挡。
- 深度缺失严重。
- 传送带表面反光。
- 外参方向或传送带姿态与配置不一致。

这里不能把“YOLO bbox 没有覆盖传送带”当成整条流程失败。YOLO 框的主要任务是找到箱体，bbox 内没有足够传送带点是正常情况。

处理规则是：

1. 保留已经完成 SOR 和箱体候选聚类的点云。
2. 将 `support_plane_valid` 设为 `false`。
3. 将 `support_plane` 设为 `None`，不能把一个不可靠的平面模型传给优化器。
4. 继续进入箱体位置优化阶段。
5. 优化器不计算支撑平面残差 $J_{support}$。
6. 结果置信度降低；如果仅靠剩余点云无法唯一约束中心，则由最终质量门控返回失败。

也就是说，支撑平面是可选的辅助约束，不是优化器的必需输入：

$$
\text{optimizer input}=
\begin{cases}
(P,\,\text{bbox},\,K,\,\text{support\_plane}), & \text{support\_plane\_valid}=\text{true}\\
(P,\,\text{bbox},\,K,\,\text{None}), & \text{support\_plane\_valid}=\text{false}
\end{cases}
$$

只有在 `support_plane_valid=true` 时，才使用传送带平面约束箱体底面高度。找不到平面时仍然可以依靠可见箱体表面、固定尺寸和图像投影关系继续优化，但通常需要降低置信度，特别是降低对 $C_z$ 的信任。

## 8. 过滤阶段总流程

~~~mermaid
flowchart TD
    A[aligned depth ROI<br/>对齐深度 ROI] --> B[depth scale conversion<br/>深度单位转换]
    B --> C[range filter<br/>深度范围过滤]
    C --> D{enough valid points<br/>有效点是否足够}
    D -- no --> X[point cloud failure<br/>点云失败]
    D -- yes --> E[back projection<br/>反投影]
    E --> F[statistical outlier removal<br/>统计离群点过滤]
    F --> G[3D clustering<br/>三维聚类]
    G --> H{crate cluster valid<br/>箱体聚类是否有效}
    H -- no --> Y[cluster failure<br/>聚类失败]
    H -- yes --> I[select and merge compatible clusters<br/>选择并合并兼容聚类]
    I --> J[optional plane RANSAC<br/>可选平面 RANSAC]
    J --> K{support plane valid<br/>支撑平面有效}
    K -- no --> L[continue without plane<br/>无平面继续]
    K -- yes --> M[remove plane inliers<br/>移除平面内点并保留模型]
    L --> N[optimization input without plane<br/>不含平面的优化输入]
    M --> O[optimization input with plane<br/>含平面的优化输入]
~~~

## 9. 固定尺寸箱体位置优化

### 9.1 优化输入

优化器输入：

~~~
P                      过滤后的箱体候选点
bbox                  YOLO 图像框
K                      对齐彩色内参
support_plane         有效时为传送带平面，否则为 None
T_base_camera         原外参
Lx, Ly, Lz            已知箱体尺寸
~~~

优化输出：

$$
\mathbf{C}_{camera}=[C_x,C_y,C_z]^{\mathsf{T}},\qquad
\mathbf{C}_{base}=\mathbf{R}_{bc}\mathbf{C}_{camera}+\mathbf{t}_{bc}
$$

当支撑平面不可见时，`support_plane=None`，但点云、bbox、内参、外参和箱体尺寸仍然进入优化器。

### 9.2 箱体模型

在箱体局部坐标系中，箱体范围为：

$$
-h_x\leq q_x\leq h_x,\qquad
-h_y\leq q_y\leq h_y,\qquad
-h_z\leq q_z\leq h_z
$$

### 9.2.1 这一步到底在做什么

这里不是把相机坐标系的绝对点直接“变成另一个点”，而是回答：

```text
假设箱体中心在 C_camera，某个观测点相对于这个中心，沿箱体 X/Y/Z 轴分别有多少距离？
```

具体分为两步：

第一步，从观测点减去候选箱体中心：

$$
\mathbf{r}_i^{camera}=\mathbf{p}_i^{camera}-\mathbf{C}_{camera}
$$

此时 $\mathbf{r}_i^{camera}$ 仍然是相机坐标系中的相对向量，表示从候选中心指向观测点。

第二步，将这个相对向量投影到箱体坐标轴上：

$$
\mathbf{q}_i=\mathbf{R}_{camera\leftarrow box}^{\mathsf{T}}
\mathbf{r}_i^{camera}
$$

如果 $\mathbf{R}_{camera\leftarrow box}$ 的列向量是箱体 X/Y/Z 轴在相机坐标系中的方向，那么转置矩阵会计算相对向量在这三个轴上的坐标。因此：

```text
q_i.x：观测点相对箱体中心沿箱体 X 轴的距离
q_i.y：观测点相对箱体中心沿箱体 Y 轴的距离
q_i.z：观测点相对箱体中心沿箱体 Z 轴的距离
```

第二步是位置优化的核心。对于一个正确的箱体中心，大部分真实箱体点应满足：

$$
|q_{i,x}|\leq h_x,\qquad
|q_{i,y}|\leq h_y,\qquad
|q_{i,z}|\leq h_z
$$

如果候选中心向右移动，所有点的 $q_{i,x}$ 会整体向负方向变化；如果候选中心向相机前方移动，所有点的 $q_{i,z}$ 会整体向负方向变化。优化器就是利用这些变化寻找最合适的 $\mathbf{C}_{camera}$。

### 9.2.2 未标定且存在小姿态变化时如何计算

这里使用 $\mathbf{R}_{bc}$ 不是因为物理上保证箱体与 `base_link` 完全同轴，而是因为本方案将 `base_link` 轴向作为未标定时的名义箱体轴向：

$$
\mathbf{R}_{base\leftarrow box}^{nominal}=\mathbf{I}
$$

外参旋转 $\mathbf{R}_{bc}$ 把 camera 向量转到 `base_link`，所以名义箱体轴在相机中的表示为：

$$
\mathbf{R}_{camera\leftarrow box}^{nominal}=
\mathbf{R}_{bc}^{\mathsf{T}}
$$

代入相对向量公式：

$$
\begin{aligned}
\mathbf{q}_i
&=\left(\mathbf{R}_{camera\leftarrow box}^{nominal}\right)^{\mathsf{T}}
  (\mathbf{p}_i^{camera}-\mathbf{C}_{camera})\\
&=\mathbf{R}_{bc}
  (\mathbf{p}_i^{camera}-\mathbf{C}_{camera})
\end{aligned}
$$

这里得到的 $\mathbf{q}_i$ 是按照名义箱体轴向计算的局部坐标。实际小姿态扰动会使 $\mathbf{q}_i$ 产生小误差；该误差由鲁棒损失、必要的几何容差和最终质量门控处理，而不是增加姿态优化变量。

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

$$
\mathbf{x}=\mathbf{C}_{camera}=[C_x,C_y,C_z]^{\mathsf{T}}
$$

不优化：

~~~
箱体长度、宽度、高度
roll、pitch、yaw
箱体形状
~~~

这样可以避免在只有部分点云时同时估计 6D 姿态造成的多解问题。

## 10. 优化目标函数

推荐使用带鲁棒损失的组合目标：

$$
J(\mathbf{C})=
w_{\mathrm{contain}}J_{\mathrm{contain}}(\mathbf{C})+
w_{\mathrm{surface}}J_{\mathrm{surface}}(\mathbf{C})+
\mathbb{1}_{\mathrm{support}}w_{\mathrm{support}}J_{\mathrm{support}}(\mathbf{C})+
w_{\mathrm{image}}J_{\mathrm{image}}(\mathbf{C})
$$

其中 $\mathbb{1}_{\mathrm{support}}$ 是支撑平面有效标志：支撑平面有效时取 $1$，找不到支撑平面时取 $0$。因此“没有传送带平面”不会中断位置优化，而是明确关闭支撑平面这一项。

### 10.1 箱体包含约束

对每个点计算局部坐标 q_i，箱体外部距离向量为：

$$
\mathbf{e}_{\mathrm{out},i}
=\max\left(\left|\mathbf{q}_i\right|-\mathbf{h},\,\mathbf{0}\right),
\qquad
\mathbf{h}=[h_x,h_y,h_z]^{\mathsf{T}}
$$

其中绝对值和最大值均按分量计算。

点在箱体内部或表面时 e_out_i 为零；点在箱体外部时产生超出箱体范围的惩罚：

$$
r_{\mathrm{contain},i}=\left\|\mathbf{e}_{\mathrm{out},i}\right\|_2,
\qquad
J_{\mathrm{contain}}(\mathbf{C})=
\sum_i\rho\left(r_{\mathrm{contain},i}^{2}\right)
$$

这里的 $\rho$ 是鲁棒损失函数。普通平方损失会让少量背景点、深度飞点或聚类残留产生非常大的影响；鲁棒损失在残差较小时仍近似平方惩罚，在残差很大时降低增长速度，从而降低异常点对中心位置的拉动。

设鲁棒尺度为 $s>0$，当前可取：

$$
s=\texttt{optimizer\_robust\_loss\_scale\_m}=0.020\,\mathrm{m}
$$

则 soft-L1 的显式形式可以写成：

$$
\rho_{\mathrm{soft\text{-}L1}}(r^2;s)=
2s^2\left(\sqrt{1+\left(\frac{r}{s}\right)^2}-1\right)
$$

对于表面残差，实际使用就是：

$$
\rho_{\mathrm{soft\text{-}L1}}
\left(r_{\mathrm{surface},i}^{2};s\right)=
2s^2\left(
\sqrt{1+\left(\frac{r_{\mathrm{surface},i}}{s}\right)^2}-1
\right)
$$

如果采用 Huber 损失，其显式形式为：

$$
\rho_{\mathrm{Huber}}(r^2;s)=
\begin{cases}
r^2, & |r|\leq s,\\
2s|r|-s^2, & |r|>s.
\end{cases}
$$

因此，$r_{\mathrm{surface},i}$ 很小时保留精细的平方惩罚；超过约 $20\,\mathrm{mm}$ 后，异常点的惩罚近似按线性增长。具体实现可选择其中一种，但文档中的 $\rho(\cdot)$ 必须与实现保持一致。

该包含项的作用是：当候选箱体中心移动到使观测点落在固定尺寸箱体外部时，产生惩罚，阻止优化后的虚拟箱体无法解释真实箱体点。它不是单独的目标分割器，背景点仍需要通过聚类、平面过滤和鲁棒损失共同处理。

### 10.2 表面贴合约束

仅使用包含约束可能让箱体移动到较大范围，只要仍然包含点云即可。因此还需要让靠近箱体外表面的点贴近模型表面。

对局部点 q_i，近似计算到最近面的距离：

$$
r_{\mathrm{surface},i}=
\min\left(
\left|h_x-|q_{i,x}|\right|,
\left|h_y-|q_{i,y}|\right|,
\left|h_z-|q_{i,z}|\right|
\right)
$$

实际实现时应对不同点使用不同权重：

- 靠近外壁、底面、上沿的点：表面残差权重高。
- 位于箱体内部的点：优先使用包含约束，表面残差权重低。
- 明显背景点：由聚类剔除或用鲁棒损失降权。

组合为：

$$
J_{\mathrm{surface}}(\mathbf{C})=
\sum_i w_{\mathrm{surface},i}\rho\left(r_{\mathrm{surface},i}^{2}\right)
$$

因为目标是开口箱体，不能把所有内部深度点都强行当作外表面点，否则箱底或箱内物体会把箱体中心拉偏。

### 10.3 传送带支撑约束

设箱体 Z 轴在相机坐标系中的单位向量为：

$$
\mathbf{a}_z=\mathbf{R}_{camera\leftarrow box}
\begin{bmatrix}0\\0\\1\end{bmatrix}
$$

箱体底面中心为：

$$
\mathbf{p}_{\mathrm{bottom}}=
\mathbf{C}_{camera}-h_z\mathbf{a}_z
$$

支撑平面为：

$$
\mathbf{n}^{\mathsf{T}}\mathbf{p}+d=0
$$

要求箱体底面中心位于支撑平面上：

$$
r_{\mathrm{support}}(\mathbf{C})=
\mathbf{n}^{\mathsf{T}}\mathbf{p}_{\mathrm{bottom}}+d,
\qquad
J_{\mathrm{support}}(\mathbf{C})=
\rho\left(r_{\mathrm{support}}(\mathbf{C})^{2}\right)
$$

该项对 C_z 特别重要，可以减少深度噪声造成的高度抖动。如果支撑平面无效，则不应使用这一项，或者将结果标记为低置信度。

### 10.4 图像投影约束

#### 10.4.1 8 个角点从哪里得到

这 8 个角点不是从深度图中检测出来的，也不是 YOLO 输出的角点，而是由固定尺寸箱体模型直接生成。箱体局部坐标中的角点集合为：

$$
\mathcal{Q}_{corner}=
\left\{
\begin{bmatrix}
\sigma_xh_x\\
\sigma_yh_y\\
\sigma_zh_z
\end{bmatrix}
\;\middle|\;
\sigma_x,\sigma_y,\sigma_z\in\{-1,+1\}
\right\}
$$

对当前候选中心 $\mathbf{C}_{camera}$，将每个局部角点变换到相机坐标系：

$$
\mathbf{p}_{corner,j}^{camera}(\mathbf{C})=
\mathbf{C}_{camera}+
\mathbf{R}_{camera\leftarrow box}^{nominal}
\mathbf{q}_{corner,j},
\qquad j=1,\ldots,8
$$

其中 $\mathbf{R}_{camera\leftarrow box}^{nominal}=\mathbf{R}_{bc}^{\mathsf{T}}$ 使用名义箱体轴向生成模型角点。实际小姿态变化不作为角点生成或位置优化变量。

#### 10.4.2 角点投影

将上述 8 个模型角点投影到图像：

$$
u=\frac{f_xX}{Z}+c_x,
\qquad
v=\frac{f_yY}{Z}+c_y
$$

得到模型投影框：

$$
\mathbf{b}_{\mathrm{model}}=
[u_{\min}^{model},v_{\min}^{model},
u_{\max}^{model},v_{\max}^{model}]^{\mathsf{T}}
$$

与 YOLO bbox 比较：

$$
\mathbf{r}_{\mathrm{image}}=
\mathbf{b}_{\mathrm{model}}-\mathbf{b}_{\mathrm{yolo}}
$$

这个项只能作为弱约束，因为 YOLO 框可能只覆盖箱体可见部分，且箱体可能被图像边界截断。

如果 YOLO bbox 接触图像边界，对应方向不能使用严格等式约束，应改成不等式约束。例如右边界被截断时，只要求模型右边界不小于图像右边界，而不是强制相等。

### 10.5 总目标

完整目标可写成：

$$
\begin{aligned}
J(\mathbf{C})={}&
w_{\mathrm{contain}}\sum_i\rho\left(r_{\mathrm{contain},i}^{2}\right)\\
&+w_{\mathrm{surface}}\sum_iw_{\mathrm{surface},i}
\rho\left(r_{\mathrm{surface},i}^{2}\right)\\
&+\mathbb{1}_{\mathrm{support}}w_{\mathrm{support}}
\rho\left(r_{\mathrm{support}}^{2}\right)\\
&+w_{\mathrm{image}}\rho\left(\left\|\mathbf{r}_{\mathrm{image}}\right\|_2^{2}\right)
\end{aligned}
$$

当 `support_plane_valid=false` 时，第三行整体为零；同时不生成支撑平面初值，也不把伪造的平面参数传给优化器。

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

$$
z_{\mathrm{init}}=
\operatorname{percentile}\left(\{Z_i\},\alpha_z\right),
\qquad \alpha_z\in[0.35,0.50]
$$

不能使用单个最近点作为初值，因为最近点可能是边缘飞点。

### 11.2 支撑平面初值

若支撑平面有效，则由箱体底面接触关系约束中心位置：

$$
\mathbf{n}^{\mathsf{T}}
\left(\mathbf{C}_{camera}-h_z\mathbf{a}_z\right)+d=0
$$

该关系可以直接给出中心沿平面法向量方向的初始位置。

### 11.3 X/Y 初值

可以使用以下信息生成候选初值：

- 候选点云质心沿箱体轴方向的位置。
- 可见点云在箱体轴方向的 min/max 与半尺寸组合。
- YOLO bbox 中心像素对应的相机射线。
- 固定尺寸模型投影框与 YOLO 框的粗匹配。

推荐先生成少量候选平移，再从最优候选进行局部优化，而不是完全依赖单个质心初值。

### 11.4 “初值”到底是什么

这里的初值是优化器开始迭代时使用的第一个箱体中心猜测，记为：

$$
\mathbf{C}_{camera}^{(0)}=
[C_x^{(0)},C_y^{(0)},C_z^{(0)}]^{\mathsf{T}}
$$

它不是最终输出，也不是额外的传感器测量，而是根据当前点云、YOLO 框和可选支撑平面构造出的“从哪里开始搜索”的位置。由于目标函数是非线性的，并且部分可见时可能存在多个局部合理位置，优化器必须从一个合理区域开始，否则可能把背景点解释成箱体，或者收敛到相机后方、工作空间外的解。

一个实用的初值生成过程如下：

1. 计算 YOLO 框中心像素：

   $$
   u_c=\frac{u_{min}+u_{max}}{2},\qquad
   v_c=\frac{v_{min}+v_{max}}{2}
   $$

2. 根据相机内参计算该像素对应的视线方向：

   $$
   \tilde{\mathbf{r}}_c=
   \mathbf{K}^{-1}
   \begin{bmatrix}u_c\\v_c\\1\end{bmatrix},
   \qquad
   \mathbf{r}_c=\frac{\tilde{\mathbf{r}}_c}
   {\left\|\tilde{\mathbf{r}}_c\right\|_2}
   $$

3. 使用候选点云得到 $z_{\mathrm{init}}$，在这条射线上生成一个粗略中心。对于针孔模型，若用 $Z$ 坐标作为深度，可写成：

   $$
   \mathbf{C}_{ray}^{(0)}=
   z_{\mathrm{init}}
   \begin{bmatrix}
   (u_c-c_x)/f_x\\
   (v_c-c_y)/f_y\\
   1
   \end{bmatrix}
   $$

4. 用候选点云的鲁棒中心、可见范围与固定半尺寸修正 $C_x^{(0)}$、$C_y^{(0)}$。如果支撑平面有效，再用接触关系修正沿平面法向量的分量；如果支撑平面无效，则跳过这一步，不从不存在的平面推导高度。

5. 对可能的前后方向歧义生成少量候选初值，分别计算目标函数的初始代价，选择代价较小且满足工作空间边界的候选作为优化起点。

### 11.5 初值在哪里被使用

初值在调用有界非线性最小二乘求解器时作为 `x0` 传入：

$$
\mathbf{x}^{(0)}=\mathbf{C}_{camera}^{(0)},
\qquad
\mathbf{x}^{(k+1)}=\mathbf{x}^{(k)}+\Delta\mathbf{x}^{(k)}
$$

每一轮迭代都会根据当前中心重新计算 $\mathbf{q}_i$、包含残差、表面残差、图像投影残差以及可选的支撑残差，再决定本轮的增量 $\Delta\mathbf{x}^{(k)}$。最终收敛的 $\mathbf{x}^{(*)}$ 才是待转换到 `base_link` 并返回服务的箱体中心。

初值本身不会被当作硬约束，也不会直接写入输出；除非后续明确增加“靠近初值”的先验项，否则它只影响搜索起点、收敛速度和落入哪个局部解。

无支撑平面时，优化器仍然接收过滤后的点云、YOLO 框、内参和固定尺寸模型，只是没有 $J_{\mathrm{support}}$，也没有由平面生成的初值修正。因此该场景可以继续优化，但高度方向通常更不确定，必须交由质量门控决定是否输出。

### 11.6 有界鲁棒优化

优化器可以使用 scipy.optimize.least_squares 或等价的有界非线性最小二乘方法：

~~~
损失：soft-L1 或 Huber
边界：工作空间 ROI
最大迭代：optimizer_max_iterations
~~~

优化变量为：

$$
\mathbf{x}=\mathbf{C}_{camera}=[C_x,C_y,C_z]^{\mathsf{T}}
$$

当前预留：

~~~yaml
optimizer_max_iterations: 200
~~~

边界用于防止优化跳到相机后方、把背景平面解释成箱体，或把箱体中心放到机器人不可达区域之外。

## 12. 优化流程图

~~~mermaid
flowchart TD
    A[filtered crate candidate points<br/>过滤后的箱体候选点] --> B[read fixed dimensions<br/>读取固定箱体尺寸]
    B --> C[derive box axes in camera<br/>计算相机系中的箱体轴向]
    C --> D[build initial center<br/>生成优化初值]
    D --> E[transform points to box local coordinates<br/>转换到箱体局部坐标]
    E --> F[containment residual<br/>包含约束残差]
    E --> G[surface residual<br/>表面贴合残差]
    D --> H{support plane valid?<br/>支撑平面是否有效}
    H -- yes / 是 --> I[support residual<br/>支撑平面残差]
    H -- no / 否 --> J[skip support residual<br/>跳过支撑平面残差]
    D --> K[project model corners<br/>投影固定尺寸模型角点]
    K --> L[weak image bbox residual<br/>弱图像框残差]
    F --> M[robust weighted objective<br/>鲁棒加权目标函数]
    G --> M
    I --> M
    J --> M
    L --> M
    M --> N[bounded translation optimization<br/>有界平移优化]
    N --> O{quality gate<br/>质量门控}
    O -- fail / 失败 --> P[return detection failure<br/>返回检测失败]
    O -- pass / 通过 --> Q[camera center<br/>相机系中心]
    Q --> R[T base camera<br/>相机到 base_link 外参]
    R --> S[base link center<br/>base_link 系中心]
~~~

## 13. 质量检查和失败条件

优化完成后不能只检查优化器是否返回数值，还要检查：

~~~
1. 有效点数达到 pointcloud_min_points
2. 聚类点数和覆盖率达标
3. SOR 后点云没有全部消失
4. 支撑平面内点比例达标，或明确标记为无支撑平面
5. 箱体模型外部超出点比例不能过高
6. 表面残差 RMSE 不超过 optimizer_max_surface_rmse_m
7. 有效点内点比例达到 optimizer_min_surface_inlier_ratio
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

$$
\texttt{box\_pose.position}=\mathbf{C}_{base}
$$

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
