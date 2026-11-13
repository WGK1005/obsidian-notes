# SLAM 建图、定位与点云基础

> 收录激光惯性里程计、2D/3D SLAM、重定位以及点云处理核心基础库。

---

## 一、 激光 SLAM 与里程计（LiDAR-Inertial Odometry / SLAM）

### 1. FAST-LIO / FAST-LIO2
- **项目地址**：https://github.com/hku-mars/FAST_LIO
- **核心作用**：港大 MARS 实验室开源的高速紧耦合 LiDAR-IMU 里程计。采用反向传播去畸变与增量式 k-d 树（ikd-Tree），计算速度极快、精度高，支持机械式（Velodyne/Ouster）和固态（Livox）雷达，是当前三维建图与机器人状态估计的核心底座。

### 2. funny_lidar_slam
- **项目地址**：https://github.com/zm0612/funny_lidar_slam
- **核心作用**：基于 C++/ROS 的模块化实时多雷达 SLAM 框架。支持多品牌机械/固态雷达与 6/9 轴 IMU，内置 LOAM、优化版 ICP、增量式 NDT 等多种前端配准方法及回环/因子图融合策略，非常适合算法对比与工程二次开发。

### 3. Cartographer (ROS 2)
- **项目地址**：https://github.com/ros2/cartographer
- **核心作用**：Google 开源的经典 2D/3D 实时 SLAM 系统（ROS 2 移植版）。基于局部子图（Submap）构建与分支定界（Branch-and-Bound）回环检测，在室内 2D 栅格建图与长距离防漂移方面非常稳健。

---

## 二、 重定位与专用平台适配（Localization）

### 4. FAST_LIO_LOCALIZATION_HUMANOID
- **项目地址**：https://github.com/deepglint/FAST_LIO_LOCALIZATION_HUMANOID
- **核心作用**：格灵深瞳开源的面向人形/足式机器人（如宇树 G1）的实时激光定位与重定位方案。将 FAST-LIO 与 Open3D 点云配准结合，在强动态抖动、大视角变化场景下提供稳定的先验地图重定位能力。

---

## 三、 点云基础库与 SLAM 经典教程（Foundation & Learning）

### 5. Open3D
- **项目地址**：https://github.com/isl-org/Open3D
- **核心作用**：现代 3D 数据处理核心开源库（C++/Python）。提供体素降采样、法向量估计、FPFH 特征提取、全局配准（RANSAC/TEASER）与多尺度 ICP，是构建点云全局重定位和离线地图后处理（去噪、切片）的利器。

### 6. slambook2（《视觉 SLAM 十四讲》第二版源码）
- **项目地址**：https://github.com/gaoxiang12/slambook2
- **核心作用**：高翔博士《视觉 SLAM 十四讲》配套开源工程。系统覆盖李群李代数（Sophus）、非线性优化（Ceres / g2o）、特征提取、直接法、后端位姿图与点云拼接，是打牢 SLAM 数学基础与代码实现规范的必读仓库。
