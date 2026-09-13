#pragma once

#define _USE_MATH_DEFINES

#include <Eigen/Dense>

#include <cmath>
#include <cstddef>
#include <string>
#include <vector>

#define C_Light 299792458.0 // 光速
#define F_GPS -4.442807633e-10 // -2 * sqrt(GM_GPS) / (C_Light * C_Light)

#define FG1_GPS 1575.42E6 // L1信号频率
#define FG2_GPS 1227.60E6 // L2信号频率
#define FG1_BDS 1561.098E6 // B1信号的基准频率
#define FG3_BDS 1268.520E6 // B3信号的基准频率

/* 波长 */
#define WL1_GPS (C_Light / FG1_GPS)
#define WL2_GPS (C_Light / FG2_GPS)
#define WL1_BDS (C_Light / FG1_BDS)
#define WL3_BDS (C_Light / FG3_BDS)

#define e2_WGS84 0.00669437999013 // WGS84椭球的第一偏心率平方
#define e2_CGCS2000 0.00669438002290
#define R_WGS84 6378137.0 // WGS84椭球的长半轴 m
#define F_WGS84 1.0 / 298.257223563 // WGS84椭球的扁率
#define R_CGS2K 6378137.0 // CGCS2000椭球的长半轴 m
#define F_CGS2K 1.0 / 298.257222101 // CGCS2000椭球的扁率

#define GM_GPS 3.986005e+14 // WGS84中的地球引力常数
#define omega_e_GPS 7.2921151467e-5 // 地球自转速率
#define GM_BDS 3.986004418e+14
#define omega_e_BDS 7.2921150e-5

constexpr double DegToRad = M_PI / 180.0;
constexpr double RadToDeg = 180.0 / M_PI;

struct ImuData
{
    double time = 0.0;                // GPS seconds of week
    Eigen::Vector3d gyro = Eigen::Vector3d::Zero();   // rad/s, body frame
    Eigen::Vector3d accel = Eigen::Vector3d::Zero();  // m/s^2, body frame
};

struct RtkData
{
    double time = 0.0;      // GPS seconds of week
    double gridY = 0.0;
    double gridX = 0.0;
    double gridZ = 0.0;
    double height = 0.0;
    double heading = 0.0;   // rad
    double pitch = 0.0;     // rad
    double roll = 0.0;      // rad
    double latitude = 0.0;  // rad
    double longitude = 0.0; // rad
    Eigen::Vector3d velocityENU = Eigen::Vector3d::Zero(); // east, north, up, m/s
    int quality = 0;
};

struct GnssLeverArm
{
    double right = 0.17;    // m, GNSS antenna relative to IMU
    double forward = 0.065; // m, GNSS antenna relative to IMU
    double up = 0.0;        // m, GNSS antenna relative to IMU

    Eigen::Vector3d BodyFRD() const
    {
        return Eigen::Vector3d(forward, right, -up);
    }
};

struct IniAlignConfig
{
    double startTime = 0.0;
    double duration = 270.0;
    bool useStartTime = false;
    double initialHeading = 0.0; // rad, clockwise from north
};

struct IniAlignResult
{
    bool success = false;
    std::string message;
    std::size_t sampleCount = 0;
    double startTime = 0.0;
    double endTime = 0.0;
    double latitude = 0.0;  // rad
    double longitude = 0.0; // rad
    double height = 0.0;    // m
    Eigen::Vector3d meanGyro = Eigen::Vector3d::Zero();   // rad/s
    Eigen::Vector3d meanAccel = Eigen::Vector3d::Zero();  // m/s^2
    Eigen::Vector3d gyroBias = Eigen::Vector3d::Zero();   // rad/s
    Eigen::Vector3d accelBias = Eigen::Vector3d::Zero();  // m/s^2
    Eigen::Vector3d velocityNED = Eigen::Vector3d::Zero(); // north, east, down, m/s
    Eigen::Matrix3d Cnb = Eigen::Matrix3d::Identity();    // body frame to NED frame
    double roll = 0.0;     // rad
    double pitch = 0.0;    // rad
    double heading = 0.0;  // rad, clockwise from north
};

struct PureInsConfig
{
    double startTime = 0.0;
    double endTime = 0.0;
    bool useStartTime = false;
    bool useEndTime = false;
    Eigen::Vector3d gyroBias = Eigen::Vector3d::Zero();  // rad/s
    Eigen::Vector3d accelBias = Eigen::Vector3d::Zero(); // m/s^2
};

struct InsState
{
    double time = 0.0;
    double latitude = 0.0;  // rad
    double longitude = 0.0; // rad
    double height = 0.0;    // m
    Eigen::Vector3d velocityNED = Eigen::Vector3d::Zero(); // north, east, down, m/s
    Eigen::Matrix3d Cnb = Eigen::Matrix3d::Identity();     // body frame to NED frame
    double roll = 0.0;     // rad
    double pitch = 0.0;    // rad
    double heading = 0.0;  // rad
};

struct LooseCouplingConfig
{
    double startTime = 0.0;
    double endTime = 0.0;
    bool useStartTime = false;
    bool useEndTime = false;
    GnssLeverArm leverArm;
    double gnssPositionStd = 0.5;      // m
    double gnssVelocityStd = 0.05;     // m/s
    double gyroNoiseStd = 0.02 * DegToRad;  // rad/s/sqrt(Hz)
    double accelNoiseStd = 0.05;            // m/s^2/sqrt(Hz)
    double gyroBiasNoiseStd = 1.0e-6;       // rad/s/sqrt(Hz)
    double accelBiasNoiseStd = 1.0e-4;      // m/s^2/sqrt(Hz)
    bool useZUPT = true;
    bool useNHC = true;
    double zuptSpeedThreshold = 0.05; // m/s
    double zuptVelocityStd = 0.02;    // m/s
    double nhcMinSpeed = 0.5;         // m/s
    double nhcInterval = 0.1;         // s
    double nhcLateralStd = 0.05;      // m/s
    double nhcVerticalStd = 0.05;     // m/s
    double nhcMaxLateralResidual = 1.5; // m/s
    double nhcMaxVerticalResidual = 1.5; // m/s
};

struct AppConfig
{
    bool useRealTimeMode = false;
    bool runPureINS = true;
    bool runForwardFilter = true;
    bool runBackwardFilter = true;
    bool runForwardBackwardSmoothing = true;
    std::string imuInput = "imu_data.txt";
    std::string rtkInput = "rtk_result.pos";
    std::string pureInsOutput = "output/pure_ins_result.nav";
    std::string forwardOutput = "output/loose_coupled_forward.nav";
    std::string backwardOutput = "output/loose_coupled_backward.nav";
    std::string smoothedOutput = "output/loose_coupled_smoothed.nav";
    std::string realTimeOutput = "output/loose_coupled_realtime.nav";
    IniAlignConfig alignConfig;
    LooseCouplingConfig looseConfig;
};

struct RealTimeProcessSummary
{
    bool success = false;
    std::string message;
    std::size_t imuLines = 0;
    std::size_t rtkLines = 0;
    std::size_t outputStates = 0;
    std::size_t gnssUpdates = 0;
    std::size_t zuptUpdates = 0;
    std::size_t nhcUpdates = 0;
    IniAlignResult alignment;
    InsState finalState;
};

AppConfig LoadAppConfig(const std::string& filePath);
std::vector<ImuData> LoadImuData(const std::string& filePath);
std::vector<RtkData> LoadRtkData(const std::string& filePath);
bool AverageRtkData(
    const std::vector<RtkData>& rtkData,
    double startTime,
    double endTime,
    RtkData& averageRtk,
    std::size_t& sampleCount);
double NormalGravity(double latitude, double height);
RtkData GnssPositionToImuPosition(
    const RtkData& gnssPosition,
    const Eigen::Matrix3d& Cnb,
    const GnssLeverArm& leverArm = GnssLeverArm());
RtkData ImuPositionToGnssPosition(
    const RtkData& imuPosition,
    const Eigen::Matrix3d& Cnb,
    const GnssLeverArm& leverArm = GnssLeverArm());

IniAlignResult InitialAlignment(
    const std::vector<ImuData>& imuData,
    const RtkData& initRtk,
    const IniAlignConfig& config = IniAlignConfig());

std::vector<InsState> RunPureINS(
    const std::vector<ImuData>& imuData,
    const IniAlignResult& initialState,
    const PureInsConfig& config = PureInsConfig());
bool SavePureINSResult(
    const std::vector<InsState>& states,
    const std::string& filePath);
std::vector<InsState> RunLooseCoupled(
    const std::vector<ImuData>& imuData,
    const std::vector<RtkData>& rtkData,
    const IniAlignResult& initialState,
    const LooseCouplingConfig& config = LooseCouplingConfig());
std::vector<InsState> RunLooseCoupledBackward(
    const std::vector<ImuData>& imuData,
    const std::vector<RtkData>& rtkData,
    const IniAlignResult& initialBias,
    const InsState& endState,
    const LooseCouplingConfig& config = LooseCouplingConfig());
bool SaveLooseCoupledResult(
    const std::vector<InsState>& states,
    const std::string& filePath);
std::vector<InsState> FuseForwardBackward(
    const std::vector<InsState>& forwardStates,
    const std::vector<InsState>& backwardStates,
    double forwardWeight = 0.5);
bool RunRealTimeLooseCoupling(
    const std::string& imuFilePath,
    const std::string& rtkFilePath,
    const AppConfig& config,
    RealTimeProcessSummary& summary);
void StartRealTimePlot();
void AddRealTimePlotPoint(double time, double latitude, double longitude);
void StopRealTimePlot();

std::string ResolveProjectDataPath(const std::string& fileName);
