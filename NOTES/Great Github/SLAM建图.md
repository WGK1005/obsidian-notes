# SLAM 建图、定位与点云基础

> 收录激光惯性里程计、多模态融合 SLAM、2D/3D 建图、重定位以及点云处理核心基础库。
[[Great Github/说明|说明]]
---

## 一、 激光与多模态 SLAM / 里程计（LiDAR & Multi-Modal SLAM）

### 1. FAST-LIO / FAST-LIO2
- **项目地址**：https://github.com/hku-mars/FAST_LIO
- **核心作用**：港大 MARS 实验室开源的高速紧耦合 LiDAR-IMU 里程计。采用反向传播去畸变与增量式 k-d 树（ikd-Tree），计算速度极快、精度高，支持机械式（Velodyne/Ouster）和固态（Livox）雷达，是当前三维建图与机器人状态估计的核心底座。
- **关联解析**：[[30_SLAM建图_FAST-LIO2紧耦合激光惯导核心剖析]]

### 2. FAST-LIVO2（激光-视觉-惯性多模态紧耦合直接法建图）
- **项目地址**：https://github.com/hku-mars/FAST-LIVO2
- **核心作用**：港大 MARS 实验室开源的旗舰级直接法激光-惯性-视觉紧耦合里程计与建图系统（LIVO）。在 FAST-LIO2 的基础上引入直接法视觉光度约束（Direct Photometric Error），无需耗时的特征点提取与描述子匹配，在单一迭代卡尔曼滤波（IEKF）框架下直接融合雷达点面几何残差与图像 Patch 光度残差。
- **核心特性与机制**：
  - **极致鲁棒的双向互补**：在暗光、无纹理场景由 LIO 几何主导；在长廊、隧道等几何退化场景由 VIO 视觉纹理兜底约束，彻底解决传统单一传感器漂移难题。
  - **高质量实时真彩着色点云**：建图过程中将相机图像实时投影至 ikd-Tree 体素三维地图点，保存带 RGB 真实色彩的全局高精度点云（`all_raw_points.pcd` / `all_downsampled_points.pcd`）。
  - **高兼容性硬件生态**：原生适配固态雷达（如 Livox Mid-70、Avia、Mid-360）与机械雷达，支持针孔及鱼眼多种工业/RGBD 相机。
- **实战工程落地与调优避坑（实车血泪经验）**：
  1. **图像降频防阻塞**：VIO 光度跟踪计算耗费多核算力，实车建议在适配桥接层（`sensor_sync_bridge`）设置 `max_img_hz: 10.0`，杜绝因高频图像堵塞导致雷达 LIO 掉帧。
  2. **曝光估计与硬件 AE 互斥**：若工业相机开启了硬件自动曝光（Auto Exposure），必须在参数中将 `exposure_estimate_en` 设为 `false`；否则算法端与硬件端相互拉扯导致曝光增益参数塌陷，引发静止时 VIO 与 LIO 姿态角互顶发散。
  3. **静止与稀疏退化保护**：针对 Mid-70 等非重复扫描雷达，在静止或局部遮挡时，当帧有效平面数不足（如 `effct_feat_num_ < 6`）易导致协方差矩阵奇异，必须在求解前增加特征数断言跳过更新；同时室内场景推荐适当收紧 IMU 协方差（`gyr_cov: 0.01`, `acc_cov: 0.1`）并将体素放大至 `0.35~0.40m`。
  4. **优雅写盘触发**：非交互式 SSH 运行模式下，通过向主节点发送 `pkill -INT -x fastlivo_mappin` 信号优雅跳出主循环，确保千万级点云完整存盘。
- **硬件档案联动**：[[03_大疆览沃_Livox_Mid-70_固态激光雷达从零上手与避坑实战]]

### 3. funny_lidar_slam
- **项目地址**：https://github.com/zm0612/funny_lidar_slam
- **核心作用**：基于 C++/ROS 的模块化实时多雷达 SLAM 框架。支持多品牌机械/固态雷达与 6/9 轴 IMU，内置 LOAM、优化版 ICP、增量式 NDT 等多种前端配准方法及回环/因子图融合策略，非常适合算法对比与工程二次开发。

### 4. Cartographer (ROS 2)
- **项目地址**：https://github.com/ros2/cartographer
- **核心作用**：Google 开源的经典 2D/3D 实时 SLAM 系统（ROS 2 移植版）。基于局部子图（Submap）构建与分支定界（Branch-and-Bound）回环检测，在室内 2D 栅格建图与长距离防漂移方面非常稳健。

---

## 二、 重定位与专用平台适配（Localization）

### 5. FAST_LIO_LOCALIZATION_HUMANOID
- **项目地址**：https://github.com/deepglint/FAST_LIO_LOCALIZATION_HUMANOID
- **核心作用**：格灵深瞳开源的面向人形/足式机器人（如宇树 G1）的实时激光定位与重定位方案。将 FAST-LIO 与 Open3D 点云配准结合，在强动态抖动、大视角变化场景下提供稳定的先验地图重定位能力。

---

## 三、 点云基础库与 SLAM 经典教程（Foundation & Learning）

### 6. Open3D
- **项目地址**：https://github.com/isl-org/Open3D
- **核心作用**：现代 3D 数据处理核心开源库（C++/Python）。提供体素降采样、法向量估计、FPFH 特征提取、全局配准（RANSAC/TEASER）与多尺度 ICP，是构建点云全局重定位和离线地图后处理（去噪、切片）的利器。

### 7. slambook2（《视觉 SLAM 十四讲》第二版源码）
- **项目地址**：https://github.com/gaoxiang12/slambook2
- **核心作用**：高翔博士《视觉 SLAM 十四讲》配套开源工程。系统覆盖李群李代数（Sophus）、非线性优化（Ceres / g2o）、特征提取、直接法、后端位姿图与点云拼接，是打牢 SLAM 数学基础与代码实现规范的必读仓库。
