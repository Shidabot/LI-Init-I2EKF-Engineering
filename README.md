# lidar-imu-calibration-engineer

[English](#english) | [中文](#中文)

## English

An engineering-oriented implementation for online LiDAR–IMU calibration and odometry in real robotic systems.

> **Project scope**
>
> This repository is an engineering integration and reliability-improvement project. It does not claim a new filtering theory, calibration model, or research algorithm. The core methods originate from existing open-source LiDAR–IMU initialization, I2EKF-LO, and FAST-LIO implementations. This edition focuses on integrating an I2EKF-style dual-iteration LiDAR frontend into the calibration stage and fixing state-management, numerical-stability, boundary-checking, and configuration issues that affect deployment.

### Engineering goals

- Use I2EKF-style LiDAR-only odometry during calibration: inner iterations update point-to-plane measurements, while outer iterations re-undistort the scan using the updated poses.
- Switch to tightly coupled LiDAR–IMU propagation and online refinement only after calibration passes quality checks.
- Reject weak excitation, degenerate motion, non-finite solutions, unusable Ceres results, and invalid sensor data explicitly.
- Support common Livox, Velodyne, Ouster, Hesai, and RoboSense configurations.

### Engineering changes

#### I2EKF calibration frontend

- Corrected the millisecond/second mismatch in point timestamps.
- Added an explicit iteration budget for outer scan re-undistortion and inner IEKF updates.
- Added configurable outer rotation/translation convergence thresholds and point-to-plane measurement variance.
- Added optional adaptive constant-velocity process noise.

#### State and numerical safety

- Roll back the current-frame state when there are too few effective points or when the gain/state increment is non-finite.
- Prevent invalid measurements from entering the map or calibration dataset.
- Use the Joseph covariance update and explicitly symmetrize the result.
- Replace fixed-size point-selection buffers that could overflow.
- Correct stale point-cloud reuse, velocity-frame conversion, and covariance semantics during the transition to LIO.

#### Calibration quality gates

- Check both rotational and translational observability. Pure rotation may start data collection, but translational degeneracy cannot complete initialization.
- Validate temporal correlation, Ceres solver status, translation-Hessian conditioning, gravity, and IMU biases.
- Remain in LiDAR-only mode after a rejected initialization instead of applying unreliable parameters.
- Expose accelerometer-bias, temporal-correlation, and observability thresholds as configuration parameters.

#### Input and preprocessing robustness

- Fix IMU-state copy overflow, zero-time-step division, and empty-buffer access.
- Guard empty point clouds and short scan lines for Velodyne, Ouster, and RoboSense inputs.
- Correct a preprocessing distance-threshold initialization error.
- Validate critical parameters at startup.

### Processing pipeline

```text
LiDAR packets
     │
     ▼
Preprocessing and frame cutting
     │
     ▼
I2EKF-style LiDAR-only odometry
  ├─ inner loop: iterated point-to-plane update
  └─ outer loop: scan re-undistortion
     │
     ▼
Temporal offset, extrinsic, gravity, and bias initialization
     │ quality gates passed
     ▼
LiDAR–IMU odometry and online refinement
```

### Dependencies

- Ubuntu 18.04 or newer
- ROS Melodic or Noetic (ROS 1)
- PCL 1.8+
- Eigen 3.3.4+
- Ceres Solver 2.0 recommended
- `livox_ros_driver`, required by the inherited message dependency
- OpenMP

### Build

```bash
cd ~/catkin_ws/src
git clone https://github.com/Shidabot/lidar-imu-calibration-engineer.git
cd ..
catkin_make -DCMAKE_BUILD_TYPE=Release
source devel/setup.bash
```

### Run

Before running, update the appropriate `config/*.yaml` file:

- `common/lid_topic` and `common/imu_topic`
- `common/mean_acc_norm`
- LiDAR and IMU noise parameters
- `initialization/*` quality thresholds
- `mapping/max_iteration`
- `i2ekf_frontend/*`

Livox Avia example:

```bash
roslaunch lidar_imu_init livox_avia.launch
```

Other launch files cover Livox Horizon/Mid-360, Velodyne, Ouster, Hesai PandarXT, and RoboSense.

Keep the sensor stationary for about five seconds after startup to build the initial map. Then provide sufficient multi-axis rotation and translation. If the program reports insufficient observability, continue exciting the system rather than treating the intermediate output as a valid calibration result. Results are written to `result/Initialization_result.txt` by default.

### I2EKF parameters

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

- `max_iteration`: maximum IEKF updates in each outer iteration.
- `max_undistort`: maximum scan re-undistortion iterations.
- `lidar_cov`: point-to-plane measurement variance.
- `outer_*_converge_*`: early-stop thresholds for the outer loop.
- `adaptive_cov_enable`: enables adaptive process noise.

Set `i2ekf_frontend/enable: false` to use the legacy single-undistortion FAST-LO path.

### Calibration quality parameters

```yaml
initialization:
    max_acc_bias: 1.0
    min_translation_eigenvalue: 1.0e-6
    max_translation_condition: 1000000.0
    min_time_correlation: 0.2
```

These defaults are engineering starting points, not universal theoretical constants. Validate them against the target IMU noise, mechanical mounting, motion range, and environment.

### Validation status and limitations

- YAML type checks and ROS Launch XML parsing have passed.
- Critical buffer, finite-matrix, measurement-rollback, and state-transition paths received static checks.
- The release-preparation environment did not contain a complete ROS 1/OpenMP toolchain, so end-to-end compilation and hardware regression were not completed there.
- Before deployment, compile in the target ROS/Ubuntu environment and perform rosbag replay, static-calibration comparison, and hardware stress tests.
- This is not a functional-safety component and must not be placed directly in an unvalidated safety-critical control path.

### License

This repository is distributed under [GNU GPL v2](LICENSE). Retain applicable copyright, license, and attribution notices when using, modifying, or redistributing the code.

---

## 中文

面向真实机器人系统的 LiDAR–IMU 在线标定与里程计工程实现。

> **项目定位**
>
> 本仓库属于工程集成与可靠性改进，不提出新的滤波理论、标定模型或科研算法。核心方法来自现有的开源 LiDAR–IMU 初始化、I2EKF-LO 与 FAST-LIO 实现。本工程重点是将 I2EKF 风格的双迭代 LiDAR 前端接入标定阶段，并修复影响实际部署的状态管理、数值稳定性、边界检查与配置问题。

### 工程目标

- 标定阶段使用 I2EKF 风格 LiDAR-only odometry：内层迭代更新点到平面量测，外层根据新位姿重新进行扫描去畸变。
- 只有标定结果通过质量检查后，才切换到 LiDAR–IMU 紧耦合传播与在线优化。
- 显式拒绝弱激励、退化运动、非有限解、无效 Ceres 结果和异常传感器数据。
- 支持常见的 Livox、Velodyne、Ouster、Hesai 和 RoboSense 配置。

### 工程修改

#### I2EKF 标定前端

- 修正点时间戳毫秒与秒的单位不一致。
- 明确外层扫描重新去畸变和内层 IEKF 更新的迭代预算。
- 增加可配置的外层旋转/平移收敛阈值和点到平面量测方差。
- 增加可选的自适应恒速度过程噪声。

#### 状态与数值安全

- 有效点过少，或增益、状态增量出现非有限值时，回滚本帧状态。
- 无效量测不写入地图，也不进入标定数据集。
- 协方差采用 Joseph 形式更新，并显式进行对称化。
- 替换可能越界的固定长度点选择缓存。
- 修复切换到 LIO 时复用旧点云、速度坐标系转换错误和协方差语义错误。

#### 标定质量门限

- 同时检查旋转和平移可观性。纯旋转可以启动数据采集，但平移退化时不能错误完成初始化。
- 检查时间相关性、Ceres 求解状态、平移 Hessian 条件、重力和 IMU 偏置。
- 初始化被拒绝后保持 LiDAR-only 模式，避免应用不可靠参数。
- 加速度计偏置、时间相关性和可观性阈值均可配置。

#### 输入与预处理健壮性

- 修复 IMU 状态复制越界、零时间间隔除法和空缓存访问。
- 防止 Velodyne、Ouster 与 RoboSense 空点云和过短扫描线越界。
- 修复预处理距离阈值初始化错误。
- 启动时校验关键配置参数。

### 处理流程

```text
LiDAR 数据
     │
     ▼
预处理与分帧
     │
     ▼
I2EKF 风格 LiDAR-only 里程计
  ├─ 内层：迭代点到平面更新
  └─ 外层：扫描重新去畸变
     │
     ▼
时间偏移、外参、重力与偏置初始化
     │ 通过质量门限
     ▼
LiDAR–IMU 里程计与在线优化
```

### 依赖

- Ubuntu 18.04 或更新版本
- ROS Melodic 或 Noetic（ROS 1）
- PCL 1.8+
- Eigen 3.3.4+
- 推荐 Ceres Solver 2.0
- `livox_ros_driver`，继承的消息依赖仍需要该包
- OpenMP

### 构建

```bash
cd ~/catkin_ws/src
git clone https://github.com/Shidabot/lidar-imu-calibration-engineer.git
cd ..
catkin_make -DCMAKE_BUILD_TYPE=Release
source devel/setup.bash
```

### 运行

运行前修改对应的 `config/*.yaml`：

- `common/lid_topic` 与 `common/imu_topic`
- `common/mean_acc_norm`
- LiDAR 和 IMU 噪声参数
- `initialization/*` 标定质量阈值
- `mapping/max_iteration`
- `i2ekf_frontend/*`

以 Livox Avia 为例：

```bash
roslaunch lidar_imu_init livox_avia.launch
```

其他 Launch 文件支持 Livox Horizon/Mid-360、Velodyne、Ouster、Hesai PandarXT 和 RoboSense。

启动后先保持传感器静止约 5 秒以建立初始地图，再进行充分的多轴旋转和平移运动。如果程序提示可观性不足，应继续激励系统，不要把中间输出当作有效标定结果。结果默认写入 `result/Initialization_result.txt`。

### I2EKF 参数

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

- `max_iteration`：每次外层迭代中的最大 IEKF 更新次数。
- `max_undistort`：最大扫描重新去畸变次数。
- `lidar_cov`：点到平面量测方差。
- `outer_*_converge_*`：外层迭代提前停止阈值。
- `adaptive_cov_enable`：是否启用自适应过程噪声。

设置 `i2ekf_frontend/enable: false` 可使用原有的单次去畸变 FAST-LO 路径。

### 标定质量参数

```yaml
initialization:
    max_acc_bias: 1.0
    min_translation_eigenvalue: 1.0e-6
    max_translation_condition: 1000000.0
    min_time_correlation: 0.2
```

这些默认值只是工程起点，不是适用于所有设备的理论常数。应结合目标 IMU 噪声、机械安装、运动范围和环境进行验证。

### 验证状态与限制

- YAML 类型检查和 ROS Launch XML 解析已经通过。
- 已对关键缓存、矩阵有限性、量测失败回滚和状态切换路径进行静态检查。
- 发布准备环境缺少完整的 ROS 1/OpenMP 工具链，因此尚未在该环境完成端到端编译和实机回归。
- 工程部署前必须在目标 ROS/Ubuntu 环境完成编译、rosbag 回放、静态标定对比和实机压力测试。
- 本项目不是功能安全组件，不应直接用于未经验证的安全关键控制链路。

### 许可证

本仓库使用 [GNU GPL v2](LICENSE)。使用、修改或再分发代码时，应保留适用的版权、许可证和署名声明。
