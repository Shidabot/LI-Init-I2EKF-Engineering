# LI-Init I2EKF Engineering Edition

面向真实机器人系统的 LiDAR–IMU 在线初始化与里程计工程实现。

> **项目定位**
>
> 本仓库是工程集成与可靠性改进项目，不宣称提出新的滤波理论、标定模型或科研算法。核心方法来自上游 [LiDAR_IMU_Init](https://github.com/hku-mars/LiDAR_IMU_Init)、[I2EKF-LO](https://github.com/Shidabot/I2EKF-LO) 及 FAST-LIO2。本仓库的工作重点是把 I2EKF 风格的双迭代 LiDAR 前端接入标定阶段，并修复影响实际部署的状态管理、数值稳定性、边界检查和配置问题。

## 工程目标

- 标定阶段使用 I2EKF 风格 LiDAR-only odometry：内层迭代更新点到平面量测，外层根据新位姿重新进行扫描去畸变。
- 标定成功后切换到原有 LiDAR–IMU 紧耦合传播与在线优化流程。
- 对弱激励、退化运动、非有限解、无效 Ceres 结果和异常传感器数据进行显式拒绝。
- 支持 Livox、Velodyne、Ouster、Hesai 和 RoboSense 等常见 LiDAR 配置。

## 与上游版本相比的工程修改

### I2EKF 标定前端

- 修正点时间戳毫秒/秒单位不一致。
- 实现外层去畸变与内层 IEKF 更新的明确迭代预算。
- 支持外层旋转、平移收敛阈值和可配置点到平面量测方差。
- 支持可选的自适应恒速度过程噪声。

### 状态与数值安全

- 量测点过少、增益或状态增量非有限时回滚本帧状态。
- 无效量测不写入地图，也不进入标定数据集。
- 协方差更新使用 Joseph 形式并强制对称化。
- 修复固定长度点选择数组可能越界的问题。
- 修复初始化切换时旧点云复用、速度坐标系转换和协方差语义问题。

### 标定质量控制

- 同时检查旋转与平移可观性；纯旋转也可以触发采集，但平移退化时不会错误完成初始化。
- 检查时间互相关强度、Ceres 求解状态、平移 Hessian 条件数、重力与偏置有限性。
- 初始化失败时保留在 LiDAR-only 模式，避免将不可信参数带入 LIO。
- 加速度计偏置边界、时间相关性和可观性阈值均可配置。

### 输入与预处理健壮性

- 修复 IMU 状态复制越界、零时间间隔除法和空缓存访问。
- 修复 Velodyne、Ouster、RoboSense 空点云及短扫描线越界。
- 修复预处理距离阈值初始化错误。
- 增加关键配置参数的启动时校验。

## 处理流程

```text
LiDAR packets
     │
     ▼
Preprocess / frame cutting
     │
     ▼
I2EKF-style LiDAR-only odometry
  ├─ inner: iterated point-to-plane update
  └─ outer: scan re-undistortion
     │
     ▼
LiDAR–IMU temporal / extrinsic / gravity / bias initialization
     │ quality gates passed
     ▼
LiDAR–IMU odometry and online refinement
```

## 依赖

- Ubuntu 18.04 或更新版本
- ROS Melodic/Noetic（ROS 1）
- PCL 1.8+
- Eigen 3.3.4+
- Ceres Solver 2.0（推荐）
- `livox_ros_driver`（上游消息依赖要求该包可被找到）
- OpenMP

## 构建

```bash
cd ~/catkin_ws/src
git clone https://github.com/Shidabot/LI-Init-I2EKF-Engineering.git
cd ..
catkin_make -DCMAKE_BUILD_TYPE=Release
source devel/setup.bash
```

## 运行

先修改对应的 `config/*.yaml`：

- `common/lid_topic`、`common/imu_topic`
- `common/mean_acc_norm`
- LiDAR 与 IMU 的量测噪声参数
- `initialization/*` 标定质量阈值
- `mapping/max_iteration`
- `i2ekf_frontend/*`

以 Livox Avia 为例：

```bash
roslaunch lidar_imu_init livox_avia.launch
```

其他设备可选择 `livox_horizon.launch`、`livox_mid360.launch`、`velodyne.launch`、`ouster.launch`、`hesai_pandarXT.launch` 或 `robosense.launch`。

启动后先静止约 5 秒建立初始地图，再进行多轴旋转和带平移的充分激励。若程序提示平移或旋转可观性不足，应继续运动，不要把尚未通过质量门限的输出作为标定结果。结果默认写入 `result/Initialization_result.txt`。

## I2EKF 参数

```yaml
mapping:
    max_iteration: 10

i2ekf_frontend:
    enable: true
    max_undistort: 3
    lidar_cov: 0.001
    outer_rot_converge_deg: 1.0
    outer_trans_converge_cm: 1.0
    adaptive_cov_enable: false
```

- `max_iteration`：每次外层去畸变中的最大 IEKF 更新次数。
- `max_undistort`：最大外层重新去畸变次数。
- `lidar_cov`：点到平面量测方差。
- `outer_*_converge_*`：外层提前收敛阈值。
- `adaptive_cov_enable`：是否启用自适应过程噪声。

关闭 `i2ekf_frontend/enable` 可回退到上游单次去畸变的 FAST-LO 路径。

## 标定质量参数

```yaml
initialization:
    max_acc_bias: 1.0
    min_translation_eigenvalue: 1.0e-6
    max_translation_condition: 1000000.0
    min_time_correlation: 0.2
```

这些默认值只是工程起点，不是适用于所有传感器的理论常数。应结合 IMU 噪声、安装刚度、运动范围和场景结构进行回放验证。

## 验证状态与限制

- 已完成 YAML 类型检查和 ROS Launch XML 解析检查。
- 已对关键缓冲区、矩阵有限性、量测失败回滚及状态切换路径进行静态检查。
- 发布前所用 macOS 环境缺少完整 ROS 1/OpenMP 工具链，因此仓库版本尚未在该环境完成端到端编译和实机回归。
- 工程部署前必须使用目标 ROS/Ubuntu 环境完成编译、rosbag 回放、静态标定对比和实机压力测试。
- 这不是功能安全组件，不应直接用于未经验证的安全关键控制链路。

## 上游与署名

本项目是基于现有开源项目的派生工程版本：

- [HKU-MaRS/LiDAR_IMU_Init](https://github.com/hku-mars/LiDAR_IMU_Init)
- [I2EKF-LO](https://github.com/Shidabot/I2EKF-LO)
- [FAST-LIO2](https://github.com/hku-mars/FAST_LIO)
- [ikd-Tree](https://github.com/hku-mars/ikd-Tree)

LI-Init 原作者包括 Fangcheng Zhu、Yunfan Ren、Wei Xu、Yixi Cai 等。使用相关算法时请引用原论文：

```bibtex
@inproceedings{zhu2022robust,
  title={Robust real-time lidar-inertial initialization},
  author={Zhu, Fangcheng and Ren, Yunfan and Zhang, Fu},
  booktitle={2022 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)},
  pages={3948--3955},
  year={2022},
  organization={IEEE}
}
```

本仓库的工程修改不改变上游论文与算法的原创归属，也不将集成、调参、异常处理或代码修复描述为新的科研贡献。

## License

继承上游项目许可，本仓库以 [GNU GPL v2](LICENSE) 发布。使用、修改和再分发时请遵守 GPLv2，并保留上游版权、许可与署名信息。
