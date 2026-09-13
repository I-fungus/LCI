# LCI

本项目用于实现 GNSS/INS 松组合程序。当前已完成 IMU、RTK 结果读取和惯导静态初始对准算法。

## 项目文件

- `LCI_Structs.h`：公共常量、结构体、函数声明。
- `IniAlign.cpp`：数据读取、RTK 窗口平均、正常重力模型、初始对准算法实现。
- `PureINS.cpp`：GNSS/IMU 杆臂位置转换和纯惯导机械编排实现。
- `main.cpp`：程序入口，读取 `data` 目录下的数据并执行初始对准。
- `data/imu_data.txt`：IMU 原始数据。
- `data/rtk_result.pos`：外部软件计算得到的 RTK 结果。
- `output/pure_ins_result.nav`：程序运行后输出的纯惯导结果文件。
- `output/loose_coupled_result.nav`：程序运行后输出的 GNSS/INS 松组合结果文件。
- `config/lci_config.ini`：程序配置文件，用于控制纯惯导、正向滤波和反向滤波开关以及输出路径。

## 数据格式

### IMU 数据

`imu_data.txt` 每行包含：

```text
Time(sow) gx(deg/s) gy(deg/s) gz(deg/s) ax(m/s^2) ay(m/s^2) az(m/s^2)
```

程序读入后：

- 时间保持 GPS 周内秒，单位为 `s`。
- 陀螺从 `deg/s` 转为 `rad/s`。
- 加速度保持 `m/s^2`。

### RTK 数据

`rtk_result.pos` 每行包含：

```text
GPSTime Grid-Y Grid-X Grid-Z Ellip-Hgt Heading Pitch Roll Latitude Longitude Local-VE Local-VN Local-VU Q
```

程序读入后：

- 纬度、经度、航向、俯仰、横滚从 `deg` 转为 `rad`。
- 速度按 ENU 顺序保存为 `velocityENU = [VE, VN, VU]`。
- `Q` 保存为 RTK 质量标志。

## 主要结构体

### `ImuData`

保存单历元 IMU 数据。

输入来源：`imu_data.txt`

主要成员：

- `time`：GPS 周内秒，单位 `s`。
- `gyro`：机体系角速度，单位 `rad/s`。
- `accel`：机体系比力，单位 `m/s^2`。

### `RtkData`

保存单历元 RTK 结果。

输入来源：`rtk_result.pos`

主要成员：

- `time`：GPS 周内秒，单位 `s`。
- `latitude`、`longitude`：纬度、经度，单位 `rad`。
- `height`：椭球高，单位 `m`。
- `heading`、`pitch`、`roll`：姿态角，单位 `rad`。
- `velocityENU`：东、北、天速度，单位 `m/s`。
- `quality`：RTK 解质量标志。

### `IniAlignConfig`

初始对准配置。

输入：

- `startTime`：对准起始时间，单位 `s`。
- `duration`：对准时长，单位 `s`。
- `useStartTime`：是否使用指定起始时间；若为 `false`，从 IMU 首历元开始。

### `IniAlignResult`

初始对准输出结果。

输出：

- `success`：算法是否成功。
- `message`：状态信息。
- `sampleCount`：参与对准的 IMU 样本数。
- `startTime`、`endTime`：实际使用的对准时间范围。
- `latitude`、`longitude`、`height`：初始位置。
- `meanGyro`：对准窗口内平均角速度，单位 `rad/s`。
- `meanAccel`：对准窗口内平均加速度，单位 `m/s^2`。
- `gyroBias`：由静止对准段估计的陀螺零偏，单位 `rad/s`。
- `accelBias`：由静止对准段估计的加速度计零偏，单位 `m/s^2`。
- `velocityNED`：初始 NED 速度，单位 `m/s`。
- `Cnb`：从机体系到 NED 导航系的方向余弦矩阵。
- `roll`、`pitch`、`heading`：初始姿态角，单位 `rad`。

### `GnssLeverArm`

保存 GNSS 天线相对 IMU 的杆臂。

当前默认值：

- `right = 0.17 m`
- `forward = 0.065 m`
- `up = 0.0 m`

程序内部惯导机体系使用 Forward-Right-Down 顺序，`BodyFRD()` 会将杆臂转换为：

```text
[forward, right, down] = [0.065, 0.17, -0.0] m
```

### `PureInsConfig`

纯惯导递推配置。

输入：

- `startTime`：纯惯导起始时间，单位 `s`。
- `endTime`：纯惯导结束时间，单位 `s`。
- `useStartTime`：是否使用指定起始时间。
- `useEndTime`：是否使用指定结束时间。
- `gyroBias`：陀螺零偏，单位 `rad/s`。
- `accelBias`：加速度计零偏，单位 `m/s^2`。

### `InsState`

保存纯惯导递推得到的单历元导航状态。

输出：

- `time`：GPS 周内秒，单位 `s`。
- `latitude`、`longitude`、`height`：IMU 中心位置。
- `velocityNED`：NED 速度 `[VN, VE, VD]`，单位 `m/s`。
- `Cnb`：机体系到 NED 导航系方向余弦矩阵。
- `roll`、`pitch`、`heading`：姿态角，单位 `rad`。

### `LooseCouplingConfig`

松组合 Kalman 滤波配置。

输入：

- `startTime`：松组合起始时间，单位 `s`。
- `endTime`：松组合结束时间，单位 `s`。
- `useStartTime`：是否使用指定起始时间。
- `useEndTime`：是否使用指定结束时间。
- `leverArm`：GNSS 天线相对 IMU 的杆臂。
- `gnssPositionStd`：GNSS 位置观测标准差，单位 `m`。
- `gnssVelocityStd`：GNSS 速度观测标准差，单位 `m/s`。
- `gyroNoiseStd`、`accelNoiseStd`：IMU 白噪声参数。
- `gyroBiasNoiseStd`、`accelBiasNoiseStd`：IMU 零偏随机游走参数。
- `useZUPT`：是否启用零速更新。
- `useNHC`：是否启用非完整约束。
- `zuptSpeedThreshold`：零速探测速度阈值，单位 `m/s`。
- `zuptVelocityStd`：ZUPT 速度虚拟观测标准差，单位 `m/s`。
- `nhcMinSpeed`：启用 NHC 的最小速度，单位 `m/s`。
- `nhcInterval`：NHC 更新间隔，单位 `s`。
- `nhcLateralStd`、`nhcVerticalStd`：NHC 侧向/垂向速度虚拟观测标准差，单位 `m/s`。

### `AppConfig`

程序运行配置，由 `config/lci_config.ini` 读取。

主要配置项：

- `run_pure_ins`：是否运行纯惯导。
- `run_forward_filter`：是否运行正向松组合滤波。
- `run_backward_filter`：是否运行反向松组合滤波。
- `run_forward_backward_smoothing`：是否运行前后向融合。
- `align_duration`：初始对准时长，单位 `s`。
- `initial_heading_deg`：初始航向角，单位 `deg`。本程序使用 NED 航向定义，从北向顺时针为正，建议范围为 `0~360`。
- `pure_ins_output`：纯惯导输出文件路径。
- `forward_output`：正向松组合输出文件路径。
- `backward_output`：反向松组合输出文件路径。
- `smoothed_output`：前后向融合输出文件路径。
- `realtime_output`：伪实时松组合输出文件路径。
- `processing_mode`：处理模式，`offline` 表示离线处理，`realtime` 表示逐行播发 IMU/RTK 文件并实时滤波。
- `gnss_position_std`：GNSS 位置观测标准差，单位 `m`。
- `gnss_velocity_std`：GNSS 速度观测标准差，单位 `m/s`。
- `gyro_noise_std_deg`：陀螺白噪声，单位 `deg/s/sqrt(Hz)`。
- `accel_noise_std`：加速度计白噪声，单位 `m/s^2/sqrt(Hz)`。
- `gyro_bias_noise_std`：陀螺零偏随机游走，单位 `rad/s/sqrt(Hz)`。
- `accel_bias_noise_std`：加速度计零偏随机游走，单位 `m/s^2/sqrt(Hz)`。
- `lever_arm_right`、`lever_arm_forward`、`lever_arm_up`：GNSS 天线相对 IMU 的杆臂，单位 `m`。
- `use_zupt`：是否启用 ZUPT。
- `use_nhc`：是否启用 NHC。
- `zupt_speed_threshold`：ZUPT 速度阈值，单位 `m/s`。
- `zupt_velocity_std`：ZUPT 速度观测标准差，单位 `m/s`。
- `nhc_min_speed`：启用 NHC 的最小速度，单位 `m/s`。
- `nhc_interval`：NHC 更新间隔，单位 `s`。
- `nhc_lateral_std`、`nhc_vertical_std`：NHC 侧向/垂向速度观测标准差，单位 `m/s`。
- `nhc_max_lateral_residual`、`nhc_max_vertical_residual`：NHC 侧向/垂向速度残差门限，单位 `m/s`，超过门限时跳过该次 NHC 更新。

若配置文件不存在，程序会自动创建默认配置文件。

## 函数说明

### `ResolveProjectDataPath`

```cpp
std::string ResolveProjectDataPath(const std::string& fileName);
```

功能：从当前工作目录开始向上查找 `data` 文件夹，并返回指定数据文件路径。

输入：

- `fileName`：数据文件名，例如 `imu_data.txt`。

输出：

- 数据文件路径字符串。

### `LoadImuData`

```cpp
std::vector<ImuData> LoadImuData(const std::string& filePath);
```

功能：读取 IMU 原始数据。

输入：

- `filePath`：IMU 文件路径。

输出：

- `std::vector<ImuData>`：IMU 数据序列。文件打开失败或无有效数据时返回空数组。

### `LoadRtkData`

```cpp
std::vector<RtkData> LoadRtkData(const std::string& filePath);
```

功能：读取 RTK 结果文件。

输入：

- `filePath`：RTK 文件路径。

输出：

- `std::vector<RtkData>`：RTK 数据序列。文件打开失败或无有效数据时返回空数组。

### `AverageRtkData`

```cpp
bool AverageRtkData(
    const std::vector<RtkData>& rtkData,
    double startTime,
    double endTime,
    RtkData& averageRtk,
    std::size_t& sampleCount);
```

功能：对指定时间窗口内的 RTK 结果求平均，用作初始位置。

输入：

- `rtkData`：RTK 数据序列。
- `startTime`：平均起始时间，单位 `s`。
- `endTime`：平均结束时间，单位 `s`。

输出：

- `averageRtk`：窗口内 RTK 平均结果。
- `sampleCount`：参与平均的 RTK 样本数。
- 返回值：窗口内有有效 RTK 数据时返回 `true`，否则返回 `false`。

### `NormalGravity`

```cpp
double NormalGravity(double latitude, double height);
```

功能：按 GRS80 正常重力模型计算指定纬度和高程处的正常重力。

输入：

- `latitude`：纬度，单位 `rad`。
- `height`：高程，单位 `m`。

输出：

- 正常重力值，单位 `m/s^2`。

计算公式：

```text
g0 = 9.7803267715 * (1 + 0.0052790414 * sin^2(latitude)
                       + 0.0000232718 * sin^4(latitude))

g = g0 - (3.087691089e-6 - 4.397731e-9 * sin^2(latitude)) * height
        + 0.721e-12 * height^2
```

### `GnssPositionToImuPosition`

```cpp
RtkData GnssPositionToImuPosition(
    const RtkData& gnssPosition,
    const Eigen::Matrix3d& Cnb,
    const GnssLeverArm& leverArm = GnssLeverArm());
```

功能：根据 GNSS 杆臂，将 GNSS 天线位置转换为 IMU 中心位置。

输入：

- `gnssPosition`：GNSS 天线位置。
- `Cnb`：机体系到 NED 系方向余弦矩阵。
- `leverArm`：GNSS 天线相对 IMU 的杆臂。

输出：

- `RtkData`：转换后的 IMU 中心位置。

### `ImuPositionToGnssPosition`

```cpp
RtkData ImuPositionToGnssPosition(
    const RtkData& imuPosition,
    const Eigen::Matrix3d& Cnb,
    const GnssLeverArm& leverArm = GnssLeverArm());
```

功能：根据 GNSS 杆臂，将 IMU 中心位置转换为 GNSS 天线位置。

输入：

- `imuPosition`：IMU 中心位置。
- `Cnb`：机体系到 NED 系方向余弦矩阵。
- `leverArm`：GNSS 天线相对 IMU 的杆臂。

输出：

- `RtkData`：转换后的 GNSS 天线位置。

### `InitialAlignment`

```cpp
IniAlignResult InitialAlignment(
    const std::vector<ImuData>& imuData,
    const RtkData& initRtk,
    const IniAlignConfig& config = IniAlignConfig());
```

功能：执行惯导静态粗对准。

输入：

- `imuData`：IMU 数据序列。
- `initRtk`：初始 RTK 平均结果，用于提供纬度、经度和高程。
- `config`：初始对准配置，包含对准时长和用户给定初始航向角。

输出：

- `IniAlignResult`：初始位置、平均 IMU、姿态矩阵和欧拉角。

当前初始对准针对 MEMS IMU 使用水平对准：由静止段平均加速度确定横滚角和俯仰角，航向角直接使用配置文件 `initial_heading_deg`。不再使用平均陀螺估计地球自转方向，因此静止段平均陀螺直接作为初始陀螺零偏。

### `RunPureINS`

```cpp
std::vector<InsState> RunPureINS(
    const std::vector<ImuData>& imuData,
    const IniAlignResult& initialState,
    const PureInsConfig& config = PureInsConfig());
```

功能：执行纯惯性导航机械编排。

输入：

- `imuData`：IMU 数据序列。
- `initialState`：初始对准输出结果。
- `config`：纯惯导配置。

输出：

- `std::vector<InsState>`：从起始时间到结束时间的纯惯导状态序列。

### `SavePureINSResult`

```cpp
bool SavePureINSResult(
    const std::vector<InsState>& states,
    const std::string& filePath);
```

功能：将纯惯导状态序列保存到文本文件。

输入：

- `states`：纯惯导状态序列。
- `filePath`：输出文件路径。

输出：

- 返回值：保存成功返回 `true`，文件无法打开或写入失败时返回 `false`。

当前主程序默认输出：

```text
output/pure_ins_result.nav
```

输出字段：

```text
Time(s) Latitude(deg) Longitude(deg) Height(m) VN(m/s) VE(m/s) VD(m/s) Roll(deg) Pitch(deg) Heading(deg)
```

### `RunLooseCoupled`

```cpp
std::vector<InsState> RunLooseCoupled(
    const std::vector<ImuData>& imuData,
    const std::vector<RtkData>& rtkData,
    const IniAlignResult& initialState,
    const LooseCouplingConfig& config = LooseCouplingConfig());
```

功能：执行 GNSS/INS 松组合 Kalman 滤波。

输入：

- `imuData`：IMU 数据序列。
- `rtkData`：RTK/GNSS 结果序列。
- `initialState`：初始对准输出结果。
- `config`：松组合滤波配置。

输出：

- `std::vector<InsState>`：松组合导航状态序列。

滤波状态为 15 维误差状态：

```text
位置误差(3) + 速度误差(3) + 姿态误差(3) + 陀螺零偏(3) + 加速度计零偏(3)
```

GNSS 观测使用 RTK 位置和速度。更新时先用杆臂将 GNSS 天线位置转换到 IMU 中心，再构造位置、速度残差。滤波更新后会反馈修正位置、速度、姿态和 IMU 零偏。

### `ZUPT`

零速更新参考课件中的虚拟零速度观测：

```text
z = v_ZUPT - v_INS = -v_INS
H = [0 I 0 0 0]
```

当前实现使用 RTK/GNSS 速度模长进行零速探测：

```text
|v_GNSS| <= zupt_speed_threshold
```

探测为静止时，构造 NED 三维零速度观测，反馈修正速度、姿态和 IMU 零偏。可通过 `use_zupt` 开关控制。

### `NHC`

非完整约束参考课件中的车辆侧向和垂向速度为 0 的虚拟速度观测：

```text
v_body_y = 0
v_body_z = 0
```

当前实现假设 IMU 机体系与车辆坐标系近似一致，使用：

```text
v_body = Cbn * v_n
z = [ -v_body_y, -v_body_z ]^T
```

量测矩阵包含速度误差项和姿态误差项。NHC 在速度大于 `nhc_min_speed` 且距离上次更新超过 `nhc_interval` 时触发。更新前会检查 `v_body_y` 和 `v_body_z` 的残差，若超过 `nhc_max_lateral_residual` 或 `nhc_max_vertical_residual`，说明当前历元可能存在坐标系安装角、侧滑、颠簸或姿态异常，本次 NHC 不参与滤波。

侧滑、漂移、弹跳等场景不满足 NHC，应增大观测噪声、增大残差门限或关闭 `use_nhc`。如果 `use_nhc=false` 时轨迹更贴近参考轨迹，优先检查 IMU 机体系是否与车辆坐标系一致。

### `RunLooseCoupledBackward`

```cpp
std::vector<InsState> RunLooseCoupledBackward(
    const std::vector<ImuData>& imuData,
    const std::vector<RtkData>& rtkData,
    const IniAlignResult& initialBias,
    const InsState& endState,
    const LooseCouplingConfig& config = LooseCouplingConfig());
```

功能：执行反向松组合滤波。

输入：

- `imuData`：原始正向 IMU 数据序列。
- `rtkData`：原始正向 RTK/GNSS 数据序列。
- `initialBias`：提供初始零偏估计。
- `endState`：反向滤波起始状态，通常使用正向滤波末状态。
- `config`：松组合滤波配置。

输出：

- `std::vector<InsState>`：按原始时间顺序排列的反向滤波状态序列。

当前实现将 IMU 和 RTK 数据映射到反向时间轴，在反向时间轴上执行同一套松组合滤波，再映射回原始时间轴输出。该结果可用于与正向结果对比，后续可扩展为前后向平滑融合。

### `FuseForwardBackward`

```cpp
std::vector<InsState> FuseForwardBackward(
    const std::vector<InsState>& forwardStates,
    const std::vector<InsState>& backwardStates,
    double forwardWeight = 0.5);
```

功能：融合正向和反向松组合结果。

输入：

- `forwardStates`：正向滤波状态序列。
- `backwardStates`：反向滤波状态序列。
- `forwardWeight`：正向结果权重，默认 `0.5`。

输出：

- `std::vector<InsState>`：融合后的状态序列。

融合时先按时间戳匹配正向和反向状态，而不是直接按数组下标匹配。若匹配时间差超过 `0.01 s`，或正反向位置、速度、姿态差异过大，则该历元保留正向滤波结果，不强行融合。通过检查后，位置、速度、高程使用线性加权平均；姿态使用四元数球面插值，避免直接平均欧拉角造成航向跨越 `0/360 deg` 时跳变。

### `SaveLooseCoupledResult`

```cpp
bool SaveLooseCoupledResult(
    const std::vector<InsState>& states,
    const std::string& filePath);
```

功能：将松组合状态序列保存到文本文件。

当前主程序默认输出：

```text
output/loose_coupled_forward.nav
output/loose_coupled_backward.nav
output/loose_coupled_smoothed.nav
output/loose_coupled_realtime.nav
```

### `RunRealTimeLooseCoupling`

```cpp
bool RunRealTimeLooseCoupling(
    const std::string& imuFilePath,
    const std::string& rtkFilePath,
    const AppConfig& config,
    RealTimeProcessSummary& summary);
```

功能：执行伪实时松组合处理。程序不一次性加载全部数据，而是从 IMU 和 RTK 文件中逐行读取、逐行播发，并在每个 IMU 历元进行惯导递推；当 RTK 时间戳不晚于当前 IMU 状态时间时，立即进行 GNSS 松组合更新。

输入：

- `imuFilePath`：IMU 原始数据文件路径。
- `rtkFilePath`：RTK/GNSS 结果文件路径。
- `config`：应用配置，包含初始对准时长、松组合噪声、ZUPT/NHC 开关和实时输出路径。
- `summary`：输出处理统计信息。

输出：

- 返回 `true` 表示实时文件播发处理成功。
- `summary` 记录初始对准结果、读取行数、输出状态数、GNSS/ZUPT/NHC 更新次数和末状态。

实时模式通过配置文件切换：

```ini
processing_mode=realtime
realtime_output=output/loose_coupled_realtime.nav
```

实时模式只执行正向滤波。反向滤波和前后向平滑需要未来数据，属于离线后处理，因此在 `processing_mode=realtime` 时不会执行。

## 初始对准算法流程

1. 根据 `IniAlignConfig` 确定对准起止时间。
2. 对同一时间窗口内的 RTK 结果求平均，得到 GNSS 天线平均位置。
3. 在对准时间窗口内累计 IMU 数据。
4. 计算平均陀螺 `meanGyro` 和平均加速度 `meanAccel`。
5. 用 `-meanAccel` 得到机体系重力方向。
6. 由机体系重力方向计算横滚角和俯仰角。
7. 读取配置文件 `initial_heading_deg` 作为初始航向角。
8. 根据横滚、俯仰和航向生成 `Cnb`。
9. 使用 GNSS 杆臂将 GNSS 天线平均位置转换为 IMU 中心位置。
10. 将静止段平均陀螺作为初始陀螺零偏。
11. 根据平均加速度与机体系理论比力估计加速度计零偏。
12. 返回初始对准结果。

## 纯惯导算法流程

1. 使用初始对准输出的 IMU 中心位置、NED 速度和 `Cnb` 初始化导航状态。
2. 从初始对准结束时刻之后的 IMU 数据开始递推。
3. 每个 IMU 历元计算采样间隔 `dt`。
4. 当前 IMU 文件提供角速度 `rad/s` 和比力 `m/s^2`，程序先转换为角增量和速度增量：

```text
dtheta = gyro * dt
dv = accel * dt
```

5. 根据纬度、高程和速度计算地球自转角速度 `omega_ie_n` 与导航系转动角速度 `omega_en_n`。
6. 扣除陀螺和加速度计零偏。
7. 参考原 `PureINS` 工程的机械编排方法，对速度增量加入划桨补偿：

```text
dv_sculling = dv + 1/2 * dtheta x dv
              + 1/12 * (dtheta_previous x dv + dv_previous x dtheta)
```

8. 对姿态角增量加入圆锥补偿：

```text
dtheta_coning = dtheta + 1/12 * dtheta_previous x dtheta
```

9. 使用补偿后的速度增量、正常重力和科氏项更新 NED 速度。
10. 使用中值速度、中值纬度和中值高程更新纬度、经度、高程。
11. 使用补偿后的角增量更新姿态矩阵，并补偿导航系转动。
12. 从 `Cnb` 提取姿态角，并保存当前 `InsState`。

## 松组合算法流程

1. 使用初始对准输出初始化 INS 状态、IMU 零偏和 15 维误差状态协方差。
2. 每个 IMU 历元执行惯导机械编排，得到名义位置、速度和姿态。
3. 用线性化误差模型传播协方差。
4. 当 RTK/GNSS 历元到达时，将 GNSS 天线位置通过杆臂转换为 IMU 中心位置。
5. 构造位置残差和速度残差。
6. 若启用 ZUPT 且检测为静止，构造零速度虚拟观测。
7. 若启用 NHC 且车辆处于运动状态，构造侧向/垂向速度为 0 的虚拟观测。
8. 执行 Kalman 更新。
9. 将误差状态反馈到名义状态：

```text
位置、速度、姿态、陀螺零偏、加速度计零偏
```

10. 清零误差状态并保存修正后的组合导航状态。

## 前后向融合流程

1. 读取正向松组合结果序列。
2. 读取反向松组合结果序列。
3. 按时间对应历元融合位置、高程和速度。
4. 将正向和反向 `Cnb` 转为四元数。
5. 使用四元数球面插值融合姿态。
6. 输出融合后的 `output/loose_coupled_smoothed.nav`。

## 主程序流程

`main.cpp` 当前执行流程：

1. 定位 `data/imu_data.txt` 和 `data/rtk_result.pos`。
2. 调用 `LoadImuData` 和 `LoadRtkData` 读取数据。
3. 使用 RTK 首历元时间作为对准起始时间。
4. 设置对准时长为 `270 s`。
5. 调用 `AverageRtkData`，对 `270 s` 窗口内 RTK 结果求平均作为 GNSS 天线平均位置。
6. 调用 `InitialAlignment`。
7. 使用 GNSS 杆臂得到 IMU 中心初始位置。
8. 将初始对准估计的 `gyroBias` 和 `accelBias` 写入 `PureInsConfig`。
9. 调用 `RunPureINS` 执行纯惯导递推。
10. 调用 `SavePureINSResult` 保存纯惯导状态序列到 `output/pure_ins_result.nav`。
11. 根据 `config/lci_config.ini` 的开关决定是否运行正向松组合滤波。
12. 根据 `config/lci_config.ini` 的开关决定是否运行反向松组合滤波。
13. 调用 `SaveLooseCoupledResult` 分别保存正向和反向松组合状态序列。
14. 根据配置开关调用 `FuseForwardBackward` 生成前后向融合结果。
15. 打印 RTK 平均样本数、初始位置、平均 IMU、IMU 零偏、初始姿态、`Cnb`、纯惯导末状态、松组合末状态和融合末状态。

## 维护说明

本 README 应随代码实时更新：

- 新增结构体时，在“主要结构体”中补充成员含义和单位。
- 新增函数时，在“函数说明”中补充功能、输入和输出。
- 修改算法时，在“算法流程”中同步更新步骤。
- 修改数据格式时，在“数据格式”中同步更新字段顺序、单位和转换规则。
