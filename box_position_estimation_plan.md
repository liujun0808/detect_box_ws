# D435i + YOLOv8-World + Known Size Constraint 3D Box Position Estimation Plan

## 1. Project Goal

实现基于人形机器人胸部 D435i 深度相机的开口箱位置估计。

已知条件：

-   使用 YOLOv8-World 检测箱体目标。
-   即使箱体部分可见，也可以完成检测。
-   箱体尺寸固定且已知。
-   目标输出箱体三维位置，不进行 6D Pose 估计。
-   不使用 ICP 配准。
-   相机安装在人形机器人胸部，机器人每次站立姿态可能存在差异。

最终输出：

    Box Position in Robot Coordinate:

    [x, y, z]

------------------------------------------------------------------------

# 2. System Architecture

整体流程：

                     D435i Camera
                          |
              -------------------------
              |                       |
             RGB                    Depth
              |                       |
              ↓                       |
        YOLOv8-World Detection        |
              |                       |
              ↓                       ↓
           2D Bounding Box ----> Depth ROI Extraction
                                      |
                                      ↓
                              Point Cloud Generation
                                      |
                                      ↓
                             Point Cloud Filtering
                                      |
                                      ↓
                     Known Size Constraint Optimization
                                      |
                                      ↓
                             Box Center Position
                                      |
                                      ↓
                      Camera Coordinate → Robot Coordinate

------------------------------------------------------------------------

# 3. Step 1: YOLOv8-World Detection

## Input

D435i RGB Image

## Output

检测结果：

    class: box

    bbox:
    u_min
    v_min
    u_max
    v_max

    confidence

说明：

-   不要求箱体完整出现。
-   检测结果只用于确定箱体大致图像区域。
-   不直接使用 bbox 中心作为箱体中心。

------------------------------------------------------------------------

# 4. Step 2: Depth ROI Extraction

根据 YOLO bbox 从深度图提取箱体区域：

    ROI = Depth[u_min:u_max, v_min:v_max]

得到目标深度：

    Z(u,v)

------------------------------------------------------------------------

# 5. Step 3: Depth Back Projection

使用 D435i 内参：

    fx
    fy
    cx
    cy

将二维深度转换为三维点：

公式：

    X = (u - cx) * Z / fx

    Y = (v - cy) * Z / fy

    Z = depth

生成：

    Box Point Cloud:

    [x1,y1,z1]
    [x2,y2,z2]
    ...

坐标系：

    Camera Coordinate

------------------------------------------------------------------------

# 6. Step 4: Point Cloud Filtering

目的：

去除 D435i 深度噪声，提高后续优化稳定性。

## 6.1 Depth Range Filter

限制有效距离：

例如：

    0.3m < Z < 3m

去除：

-   深度异常点
-   远距离噪声

------------------------------------------------------------------------

## 6.2 Statistical Outlier Removal

使用 PCL：

    StatisticalOutlierRemoval

去除孤立点。

------------------------------------------------------------------------

## 6.3 Ground Plane Removal

如果箱子位于地面：

使用 RANSAC Plane：

    ax + by + cz + d = 0

检测地面并移除。

保留：

    Box Point Cloud

------------------------------------------------------------------------

# 7. Step 5: Known Size Constraint Optimization

## 7.1 Input

已知箱体尺寸：

    Length  = L
    Width   = W
    Height  = H

例如：

    0.6m × 0.4m × 0.3m

------------------------------------------------------------------------

## 7.2 Optimization Target

未知变量：

    Box Center:

    C = (x,y,z)

目标：

寻找一个满足尺寸约束的虚拟箱体，使其：

1.  包含观测点云。
2.  与实际点云距离最小。
3.  尺寸固定为已知尺寸。

优化目标：

    min Σ distance(Point_i, Box(C,L,W,H))

------------------------------------------------------------------------

## 7.3 Optimization Output

输出：

    Box Center in Camera Frame:

    [x,y,z]

特点：

-   不依赖完整箱体观测。
-   不需要 ICP 初始位姿。
-   利用箱体尺寸先验提高稳定性。

------------------------------------------------------------------------

# 8. Step 6: Coordinate Transformation

当前结果：

    Camera Coordinate

转换到机器人坐标：

    Robot/Base Coordinate

需要外参：

    T_robot_camera

转换：

    P_robot = T_robot_camera * P_camera

输出：

    Box Position:

    x
    y
    z

------------------------------------------------------------------------

# 9. Implementation Modules

建议代码结构：

    box_position_estimation/

    ├── detector/
    │   └── yolov8_world_detector.py
    │
    ├── camera/
    │   ├── d435i_driver.py
    │   └── camera_model.py
    │
    ├── pointcloud/
    │   ├── depth_to_cloud.py
    │   ├── filter.py
    │   └── ground_remove.py
    │
    ├── optimization/
    │   └── box_size_fitting.py
    │
    ├── calibration/
    │   └── coordinate_transform.py
    │
    ├── main.py
    │
    └── config.yaml

------------------------------------------------------------------------

# 10. Configuration

config.yaml:

``` yaml
box:
  length: 0.6
  width: 0.4
  height: 0.3

camera:
  fx:
  fy:
  cx:
  cy:

transform:
  robot_camera:
```

------------------------------------------------------------------------

# 11. Runtime Pipeline

实时运行：

    while True:

        RGB, Depth = D435i.get_frame()

        detection = YOLO.detect(RGB)

        bbox = detection.box

        cloud = depth_to_cloud(
            Depth,
            bbox
        )

        cloud = filter(cloud)

        center_camera = fit_box_size(
            cloud,
            box_dimension
        )

        center_robot = transform(
            center_camera
        )

        publish(center_robot)

------------------------------------------------------------------------

# 12. Development Priority

实现顺序：

1.  D435i RGB-D 数据获取。
2.  YOLOv8-World 箱体检测。
3.  Depth ROI 转点云。
4.  点云过滤。
5.  已知尺寸箱体优化算法。
6.  相机到机器人坐标转换。
7.  实时运行与稳定性测试。

------------------------------------------------------------------------

# 13. Expected Advantages

相比 ICP：

-   不需要初始位姿。
-   不依赖完整箱体点云。
-   适应机器人姿态变化。
-   利用固定尺寸先验。
-   更适合移动机器人场景。

最终方案：

    YOLOv8-World
            +
    D435i Depth
            +
    Known Box Size Constraint Optimization
            +
    Robot Coordinate Transform

    = Box 3D Position Estimation
