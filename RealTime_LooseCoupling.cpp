#include "LCI_Structs.h"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>

namespace
{
constexpr int kErrorStateDim = 15;

bool ParseNumericLine(const std::string& line, std::vector<double>& values)
{
    values.clear();
    std::istringstream iss(line);
    double value = 0.0;
    while (iss >> value)
    {
        values.push_back(value);
    }
    return !values.empty();
}

bool ParseImuLine(const std::string& line, ImuData& imu)
{
    std::vector<double> values;
    if (!ParseNumericLine(line, values) || values.size() < 7)
    {
        return false;
    }

    imu.time = values[0];
    imu.gyro = Eigen::Vector3d(values[1], values[2], values[3]) * DegToRad;
    imu.accel = Eigen::Vector3d(values[4], values[5], values[6]);
    return true;
}

bool ParseRtkLine(const std::string& line, RtkData& rtk)
{
    std::vector<double> values;
    if (!ParseNumericLine(line, values) || values.size() < 14)
    {
        return false;
    }

    rtk.time = values[0];
    rtk.gridY = values[1];
    rtk.gridX = values[2];
    rtk.gridZ = values[3];
    rtk.height = values[4];
    rtk.heading = values[5] * DegToRad;
    rtk.pitch = values[6] * DegToRad;
    rtk.roll = values[7] * DegToRad;
    rtk.latitude = values[8] * DegToRad;
    rtk.longitude = values[9] * DegToRad;
    rtk.velocityENU = Eigen::Vector3d(values[10], values[11], values[12]);
    rtk.quality = static_cast<int>(values[13]);
    return true;
}

bool ReadNextImu(std::ifstream& input, ImuData& imu, std::size_t& lineCount)
{
    std::string line;
    while (std::getline(input, line))
    {
        ++lineCount;
        if (ParseImuLine(line, imu))
        {
            return true;
        }
    }
    return false;
}

bool ReadNextRtk(std::ifstream& input, RtkData& rtk, std::size_t& lineCount)
{
    std::string line;
    while (std::getline(input, line))
    {
        ++lineCount;
        if (ParseRtkLine(line, rtk))
        {
            return true;
        }
    }
    return false;
}

Eigen::Matrix3d SkewSymmetric(const Eigen::Vector3d& vector)
{
    Eigen::Matrix3d matrix;
    matrix << 0.0, -vector.z(), vector.y(),
              vector.z(), 0.0, -vector.x(),
              -vector.y(), vector.x(), 0.0;
    return matrix;
}

Eigen::Matrix3d RotationVectorToMatrix(const Eigen::Vector3d& rotationVector)
{
    const double angle = rotationVector.norm();
    if (angle < 1.0e-12)
    {
        return Eigen::Matrix3d::Identity() + SkewSymmetric(rotationVector);
    }

    return Eigen::AngleAxisd(angle, rotationVector / angle).toRotationMatrix();
}

void NormalizeDcm(Eigen::Matrix3d& Cnb)
{
    Eigen::JacobiSVD<Eigen::Matrix3d> svd(Cnb, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Cnb = svd.matrixU() * svd.matrixV().transpose();
    if (Cnb.determinant() < 0.0)
    {
        Eigen::Matrix3d u = svd.matrixU();
        u.col(2) *= -1.0;
        Cnb = u * svd.matrixV().transpose();
    }
}

double PrimeVerticalRadius(double latitude)
{
    const double sinLat = std::sin(latitude);
    return R_WGS84 / std::sqrt(1.0 - e2_WGS84 * sinLat * sinLat);
}

double MeridianRadius(double latitude)
{
    const double sinLat = std::sin(latitude);
    const double denominator = 1.0 - e2_WGS84 * sinLat * sinLat;
    return R_WGS84 * (1.0 - e2_WGS84) / std::pow(denominator, 1.5);
}

void SaveEuler(InsState& state)
{
    state.roll = std::atan2(state.Cnb(2, 1), state.Cnb(2, 2));
    state.pitch = std::asin(std::clamp(-state.Cnb(2, 0), -1.0, 1.0));
    state.heading = std::atan2(state.Cnb(1, 0), state.Cnb(0, 0));
    if (state.heading < 0.0)
    {
        state.heading += 2.0 * M_PI;
    }
}

void ApplyPositionCorrection(InsState& state, const Eigen::Vector3d& deltaPositionNED)
{
    const double rm = MeridianRadius(state.latitude);
    const double rn = PrimeVerticalRadius(state.latitude);
    state.latitude += deltaPositionNED.x() / (rm + state.height);
    state.longitude += deltaPositionNED.y() / ((rn + state.height) * std::cos(state.latitude));
    state.height -= deltaPositionNED.z();
}

void ApplyErrorState(
    const Eigen::Matrix<double, kErrorStateDim, 1>& dx,
    InsState& state,
    Eigen::Vector3d& gyroBias,
    Eigen::Vector3d& accelBias)
{
    ApplyPositionCorrection(state, dx.segment<3>(0));
    state.velocityNED += dx.segment<3>(3);
    state.Cnb = (Eigen::Matrix3d::Identity() + SkewSymmetric(dx.segment<3>(6))) * state.Cnb;
    NormalizeDcm(state.Cnb);
    SaveEuler(state);
    gyroBias += dx.segment<3>(9);
    accelBias += dx.segment<3>(12);
}

template <int MeasurementDim>
void KalmanUpdate(
    const Eigen::Matrix<double, MeasurementDim, 1>& innovation,
    const Eigen::Matrix<double, MeasurementDim, kErrorStateDim>& H,
    const Eigen::Matrix<double, MeasurementDim, MeasurementDim>& R,
    InsState& state,
    Eigen::Vector3d& gyroBias,
    Eigen::Vector3d& accelBias,
    Eigen::Matrix<double, kErrorStateDim, kErrorStateDim>& P)
{
    const Eigen::Matrix<double, MeasurementDim, MeasurementDim> S = H * P * H.transpose() + R;
    const Eigen::Matrix<double, kErrorStateDim, MeasurementDim> K =
        P * H.transpose() * S.inverse();
    const Eigen::Matrix<double, kErrorStateDim, 1> dx = K * innovation;
    ApplyErrorState(dx, state, gyroBias, accelBias);

    const Eigen::Matrix<double, kErrorStateDim, kErrorStateDim> I =
        Eigen::Matrix<double, kErrorStateDim, kErrorStateDim>::Identity();
    P = (I - K * H) * P * (I - K * H).transpose() + K * R * K.transpose();
    P = 0.5 * (P + P.transpose());
}

void WriteState(std::ofstream& output, const InsState& state)
{
    output << state.time << ' '
           << state.latitude * RadToDeg << ' '
           << state.longitude * RadToDeg << ' '
           << state.height << ' '
           << state.velocityNED.x() << ' '
           << state.velocityNED.y() << ' '
           << state.velocityNED.z() << ' '
           << state.roll * RadToDeg << ' '
           << state.pitch * RadToDeg << ' '
           << state.heading * RadToDeg << '\n';
}

void PrintRealtimeState(const InsState& state)
{
    std::cout << std::fixed << std::setprecision(10)
              << state.time << ' '
              << state.latitude * RadToDeg << ' '
              << state.longitude * RadToDeg << ' '
              << state.height << ' '
              << state.velocityNED.x() << ' '
              << state.velocityNED.y() << ' '
              << state.velocityNED.z() << ' '
              << state.roll * RadToDeg << ' '
              << state.pitch * RadToDeg << ' '
              << state.heading * RadToDeg << '\n';
}

class RealTimeLooseCouplingProcessor
{
public:
    RealTimeLooseCouplingProcessor(
        const IniAlignResult& alignment,
        const LooseCouplingConfig& config,
        const ImuData& previousImu)
        : config_(config),
          previousImu_(previousImu)
    {
        state_.time = alignment.endTime;
        state_.latitude = alignment.latitude;
        state_.longitude = alignment.longitude;
        state_.height = alignment.height;
        state_.velocityNED = alignment.velocityNED;
        state_.Cnb = alignment.Cnb;
        SaveEuler(state_);

        gyroBias_ = alignment.gyroBias;
        accelBias_ = alignment.accelBias;

        P_.setZero();
        P_.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() * 10.0;
        P_.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity() * 1.0;
        P_.block<3, 3>(6, 6) = Eigen::Matrix3d::Identity() * std::pow(2.0 * DegToRad, 2);
        P_.block<3, 3>(9, 9) = Eigen::Matrix3d::Identity() * std::pow(0.05 * DegToRad, 2);
        P_.block<3, 3>(12, 12) = Eigen::Matrix3d::Identity() * 0.01;

        midLatitude_ = state_.latitude;
        midHeight_ = state_.height;
        midVelocity_ = state_.velocityNED;
        lastNhcTime_ = state_.time;
    }

    const InsState& State() const
    {
        return state_;
    }

    bool ProcessImu(const ImuData& imu, RealTimeProcessSummary& summary)
    {
        if (imu.time <= state_.time)
        {
            previousImu_ = imu;
            return false;
        }

        const double dt = imu.time - previousImu_.time;
        if (dt <= 0.0 || dt > 1.0)
        {
            previousImu_ = imu;
            return false;
        }

        PropagateInsState(imu, dt);
        PropagateCovariance(imu, dt);

        if (config_.useNHC &&
            state_.velocityNED.norm() >= config_.nhcMinSpeed &&
            state_.time - lastNhcTime_ >= config_.nhcInterval)
        {
            if (NhcUpdate())
            {
                ++summary.nhcUpdates;
                lastNhcTime_ = state_.time;
            }
        }

        previousImu_ = imu;
        return true;
    }

    void ProcessRtk(const RtkData& rtk, RealTimeProcessSummary& summary)
    {
        GnssUpdate(rtk);
        ++summary.gnssUpdates;

        const Eigen::Vector3d observedVelocityNED(
            rtk.velocityENU.y(),
            rtk.velocityENU.x(),
            -rtk.velocityENU.z());
        if (config_.useZUPT && observedVelocityNED.norm() <= config_.zuptSpeedThreshold)
        {
            ZuptUpdate();
            ++summary.zuptUpdates;
        }
    }

private:
    void PropagateInsState(const ImuData& currentImu, double dt)
    {
        const InsState previousState = state_;
        const double latitudeForUpdate = previousState.latitude * 2.0 - midLatitude_;
        const double heightForUpdate = previousState.height * 2.0 - midHeight_;
        const double vNForUpdate = previousState.velocityNED.x() * 2.0 - midVelocity_.x();
        const double vEForUpdate = previousState.velocityNED.y() * 2.0 - midVelocity_.y();
        const double vDForUpdate = previousState.velocityNED.z() * 2.0 - midVelocity_.z();

        const Eigen::Vector3d omegaIeN(
            omega_e_GPS * std::cos(latitudeForUpdate),
            0.0,
            -omega_e_GPS * std::sin(latitudeForUpdate));
        const Eigen::Vector3d omegaEnN(
            vEForUpdate / (PrimeVerticalRadius(latitudeForUpdate) + heightForUpdate),
            -vNForUpdate / (MeridianRadius(latitudeForUpdate) + heightForUpdate),
            -vEForUpdate * std::tan(latitudeForUpdate) /
                (PrimeVerticalRadius(latitudeForUpdate) + heightForUpdate));

        const Eigen::Vector3d deltaTheta = (currentImu.gyro - gyroBias_) * dt;
        const Eigen::Vector3d deltaVelocity = (currentImu.accel - accelBias_) * dt;

        Eigen::Vector3d scullingVelocity = deltaVelocity + 0.5 * deltaTheta.cross(deltaVelocity);
        Eigen::Vector3d coningRotation = deltaTheta;
        if (hasPreviousDelta_)
        {
            scullingVelocity +=
                (previousDeltaTheta_.cross(deltaVelocity) + previousDeltaVelocity_.cross(deltaTheta)) / 12.0;
            coningRotation += previousDeltaTheta_.cross(deltaTheta) / 12.0;
        }

        const Eigen::Vector3d zeta = (omegaIeN + omegaEnN) * dt;
        const Eigen::Vector3d specificForceVelocity =
            (Eigen::Matrix3d::Identity() - 0.5 * SkewSymmetric(zeta)) *
            previousState.Cnb * scullingVelocity;
        const Eigen::Vector3d gravityN(0.0, 0.0, NormalGravity(latitudeForUpdate, heightForUpdate));
        const Eigen::Vector3d velocityForUpdate(vNForUpdate, vEForUpdate, vDForUpdate);
        const Eigen::Vector3d gravityVelocity =
            (gravityN - (2.0 * omegaIeN + omegaEnN).cross(velocityForUpdate)) * dt;

        state_.velocityNED = previousState.velocityNED + specificForceVelocity + gravityVelocity;
        midVelocity_ = 0.5 * (state_.velocityNED + previousState.velocityNED);

        state_.height = previousState.height - midVelocity_.z() * dt;
        midHeight_ = 0.5 * (state_.height + previousState.height);
        state_.latitude = previousState.latitude +
            midVelocity_.x() / (MeridianRadius(previousState.latitude) + midHeight_) * dt;
        midLatitude_ = 0.5 * (state_.latitude + previousState.latitude);
        state_.longitude = previousState.longitude +
            midVelocity_.y() / ((PrimeVerticalRadius(midLatitude_) + midHeight_) * std::cos(midLatitude_)) * dt;

        state_.Cnb =
            RotationVectorToMatrix(-zeta) * previousState.Cnb * RotationVectorToMatrix(coningRotation);
        NormalizeDcm(state_.Cnb);
        state_.time = currentImu.time;
        SaveEuler(state_);

        previousDeltaTheta_ = deltaTheta;
        previousDeltaVelocity_ = deltaVelocity;
        hasPreviousDelta_ = true;
    }

    void PropagateCovariance(const ImuData& imu, double dt)
    {
        Eigen::Matrix<double, kErrorStateDim, kErrorStateDim> F =
            Eigen::Matrix<double, kErrorStateDim, kErrorStateDim>::Zero();
        F.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity();

        const Eigen::Vector3d specificForceN = state_.Cnb * (imu.accel - accelBias_);
        F.block<3, 3>(3, 6) = -SkewSymmetric(specificForceN);
        F.block<3, 3>(3, 12) = -state_.Cnb;

        const Eigen::Vector3d omegaIeN(
            omega_e_GPS * std::cos(state_.latitude),
            0.0,
            -omega_e_GPS * std::sin(state_.latitude));
        F.block<3, 3>(6, 6) = -SkewSymmetric(omegaIeN);
        F.block<3, 3>(6, 9) = -state_.Cnb;

        const Eigen::Matrix<double, kErrorStateDim, kErrorStateDim> Phi =
            Eigen::Matrix<double, kErrorStateDim, kErrorStateDim>::Identity() + F * dt;

        Eigen::Matrix<double, kErrorStateDim, kErrorStateDim> Q =
            Eigen::Matrix<double, kErrorStateDim, kErrorStateDim>::Zero();
        Q.block<3, 3>(3, 3) =
            Eigen::Matrix3d::Identity() * config_.accelNoiseStd * config_.accelNoiseStd * dt;
        Q.block<3, 3>(6, 6) =
            Eigen::Matrix3d::Identity() * config_.gyroNoiseStd * config_.gyroNoiseStd * dt;
        Q.block<3, 3>(9, 9) =
            Eigen::Matrix3d::Identity() * config_.gyroBiasNoiseStd * config_.gyroBiasNoiseStd * dt;
        Q.block<3, 3>(12, 12) =
            Eigen::Matrix3d::Identity() * config_.accelBiasNoiseStd * config_.accelBiasNoiseStd * dt;

        P_ = Phi * P_ * Phi.transpose() + Q;
        P_ = 0.5 * (P_ + P_.transpose());
    }

    void GnssUpdate(const RtkData& rtk)
    {
        const RtkData observedImuPosition = GnssPositionToImuPosition(rtk, state_.Cnb, config_.leverArm);
        const double rm = MeridianRadius(state_.latitude);
        const double rn = PrimeVerticalRadius(state_.latitude);
        const Eigen::Vector3d positionInnovation(
            (observedImuPosition.latitude - state_.latitude) * (rm + state_.height),
            (observedImuPosition.longitude - state_.longitude) *
                (rn + state_.height) * std::cos(state_.latitude),
            state_.height - observedImuPosition.height);
        const Eigen::Vector3d observedVelocityNED(
            rtk.velocityENU.y(),
            rtk.velocityENU.x(),
            -rtk.velocityENU.z());

        Eigen::Matrix<double, 6, 1> innovation;
        innovation.segment<3>(0) = positionInnovation;
        innovation.segment<3>(3) = observedVelocityNED - state_.velocityNED;

        Eigen::Matrix<double, 6, kErrorStateDim> H =
            Eigen::Matrix<double, 6, kErrorStateDim>::Zero();
        H.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
        H.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity();

        Eigen::Matrix<double, 6, 6> R = Eigen::Matrix<double, 6, 6>::Zero();
        R.block<3, 3>(0, 0) =
            Eigen::Matrix3d::Identity() * config_.gnssPositionStd * config_.gnssPositionStd;
        R.block<3, 3>(3, 3) =
            Eigen::Matrix3d::Identity() * config_.gnssVelocityStd * config_.gnssVelocityStd;

        KalmanUpdate<6>(innovation, H, R, state_, gyroBias_, accelBias_, P_);
    }

    void ZuptUpdate()
    {
        Eigen::Matrix<double, 3, 1> innovation = -state_.velocityNED;
        Eigen::Matrix<double, 3, kErrorStateDim> H =
            Eigen::Matrix<double, 3, kErrorStateDim>::Zero();
        H.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity();
        const Eigen::Matrix3d R = Eigen::Matrix3d::Identity() *
            config_.zuptVelocityStd * config_.zuptVelocityStd;

        KalmanUpdate<3>(innovation, H, R, state_, gyroBias_, accelBias_, P_);
    }

    bool NhcUpdate()
    {
        const Eigen::Matrix3d Cbn = state_.Cnb.transpose();
        const Eigen::Vector3d velocityBody = Cbn * state_.velocityNED;
        if (std::abs(velocityBody.y()) > config_.nhcMaxLateralResidual ||
            std::abs(velocityBody.z()) > config_.nhcMaxVerticalResidual)
        {
            return false;
        }

        Eigen::Matrix<double, 2, 1> innovation;
        innovation << -velocityBody.y(), -velocityBody.z();

        Eigen::Matrix<double, 2, kErrorStateDim> H =
            Eigen::Matrix<double, 2, kErrorStateDim>::Zero();
        const Eigen::Matrix3d attitudeJacobian = Cbn * SkewSymmetric(state_.velocityNED);
        H.block<2, 3>(0, 3) = Cbn.block<2, 3>(1, 0);
        H.block<2, 3>(0, 6) = attitudeJacobian.block<2, 3>(1, 0);

        Eigen::Matrix2d R = Eigen::Matrix2d::Zero();
        R(0, 0) = config_.nhcLateralStd * config_.nhcLateralStd;
        R(1, 1) = config_.nhcVerticalStd * config_.nhcVerticalStd;

        KalmanUpdate<2>(innovation, H, R, state_, gyroBias_, accelBias_, P_);
        return true;
    }

    LooseCouplingConfig config_;
    InsState state_;
    ImuData previousImu_;
    Eigen::Vector3d gyroBias_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d accelBias_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d previousDeltaTheta_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d previousDeltaVelocity_ = Eigen::Vector3d::Zero();
    bool hasPreviousDelta_ = false;
    double midLatitude_ = 0.0;
    double midHeight_ = 0.0;
    Eigen::Vector3d midVelocity_ = Eigen::Vector3d::Zero();
    double lastNhcTime_ = 0.0;
    Eigen::Matrix<double, kErrorStateDim, kErrorStateDim> P_;
};
}

bool RunRealTimeLooseCoupling(
    const std::string& imuFilePath,
    const std::string& rtkFilePath,
    const AppConfig& config,
    RealTimeProcessSummary& summary)
{
    summary = RealTimeProcessSummary();

    std::ifstream imuInput(imuFilePath);
    std::ifstream rtkInput(rtkFilePath);
    if (!imuInput.is_open() || !rtkInput.is_open())
    {
        summary.message = "Failed to open IMU or RTK data file.";
        return false;
    }

    RtkData firstRtk;
    if (!ReadNextRtk(rtkInput, firstRtk, summary.rtkLines))
    {
        summary.message = "No valid RTK data for real-time initial alignment.";
        return false;
    }

    IniAlignConfig alignConfig = config.alignConfig;
    alignConfig.startTime = firstRtk.time;
    alignConfig.useStartTime = true;
    const double alignEndTime = alignConfig.startTime + alignConfig.duration;

    std::vector<RtkData> alignRtkData;
    alignRtkData.push_back(firstRtk);
    RtkData pendingRtk;
    bool hasPendingRtk = false;
    RtkData currentRtk;
    while (ReadNextRtk(rtkInput, currentRtk, summary.rtkLines))
    {
        if (currentRtk.time > alignEndTime)
        {
            pendingRtk = currentRtk;
            hasPendingRtk = true;
            break;
        }
        alignRtkData.push_back(currentRtk);
    }

    std::vector<ImuData> alignImuData;
    ImuData previousImu;
    ImuData firstProcessingImu;
    bool hasPreviousImu = false;
    bool hasFirstProcessingImu = false;
    ImuData currentImu;
    while (ReadNextImu(imuInput, currentImu, summary.imuLines))
    {
        if (currentImu.time > alignEndTime)
        {
            firstProcessingImu = currentImu;
            hasFirstProcessingImu = true;
            break;
        }
        alignImuData.push_back(currentImu);
        previousImu = currentImu;
        hasPreviousImu = true;
    }

    if (!hasPreviousImu)
    {
        summary.message = "No valid IMU data for real-time initial alignment.";
        return false;
    }

    RtkData initRtk;
    std::size_t rtkAverageCount = 0;
    if (!AverageRtkData(alignRtkData, alignConfig.startTime, alignEndTime, initRtk, rtkAverageCount))
    {
        summary.message = "Failed to average RTK data for real-time initial alignment.";
        return false;
    }

    summary.alignment = InitialAlignment(alignImuData, initRtk, alignConfig);
    if (!summary.alignment.success)
    {
        summary.message = summary.alignment.message;
        return false;
    }

    std::vector<RtkData> bufferedFilterRtk;
    for (const RtkData& rtk : alignRtkData)
    {
        if (rtk.time > summary.alignment.endTime)
        {
            bufferedFilterRtk.push_back(rtk);
        }
    }

    namespace fs = std::filesystem;
    const fs::path outputPath(config.realTimeOutput);
    if (outputPath.has_parent_path())
    {
        fs::create_directories(outputPath.parent_path());
    }

    std::ofstream output(config.realTimeOutput);
    if (!output.is_open())
    {
        summary.message = "Failed to open real-time output file.";
        return false;
    }

    output << "Time(s) Latitude(deg) Longitude(deg) Height(m) "
           << "VN(m/s) VE(m/s) VD(m/s) Roll(deg) Pitch(deg) Heading(deg)\n";
    output << std::fixed << std::setprecision(10);

    StartRealTimePlot();

    LooseCouplingConfig lcConfig = config.looseConfig;
    lcConfig.startTime = summary.alignment.endTime;
    lcConfig.useStartTime = true;

    RealTimeLooseCouplingProcessor processor(summary.alignment, lcConfig, previousImu);
    WriteState(output, processor.State());
    ++summary.outputStates;

    std::cout << "Realtime state output every 1 s:\n"
              << "Time(s) Latitude(deg) Longitude(deg) Height(m) "
              << "VN(m/s) VE(m/s) VD(m/s) Roll(deg) Pitch(deg) Heading(deg)\n";
    PrintRealtimeState(processor.State());
    AddRealTimePlotPoint(
        processor.State().time,
        processor.State().latitude,
        processor.State().longitude);
    double nextConsoleOutputTime = std::floor(processor.State().time) + 1.0;

    std::size_t bufferedRtkIndex = 0;
    auto processReadyRtk = [&]() {
        while (bufferedRtkIndex < bufferedFilterRtk.size() &&
               bufferedFilterRtk[bufferedRtkIndex].time <= processor.State().time)
        {
            processor.ProcessRtk(bufferedFilterRtk[bufferedRtkIndex], summary);
            ++bufferedRtkIndex;
        }
        while (hasPendingRtk && pendingRtk.time <= processor.State().time)
        {
            processor.ProcessRtk(pendingRtk, summary);
            hasPendingRtk = ReadNextRtk(rtkInput, pendingRtk, summary.rtkLines);
        }
    };

    if (hasFirstProcessingImu)
    {
        if (processor.ProcessImu(firstProcessingImu, summary))
        {
            processReadyRtk();
            WriteState(output, processor.State());
            ++summary.outputStates;
            if (processor.State().time >= nextConsoleOutputTime)
            {
                PrintRealtimeState(processor.State());
                AddRealTimePlotPoint(
                    processor.State().time,
                    processor.State().latitude,
                    processor.State().longitude);
                nextConsoleOutputTime = std::floor(processor.State().time) + 1.0;
            }
        }
    }

    while (ReadNextImu(imuInput, currentImu, summary.imuLines))
    {
        if (processor.ProcessImu(currentImu, summary))
        {
            processReadyRtk();
            WriteState(output, processor.State());
            ++summary.outputStates;
            if (processor.State().time >= nextConsoleOutputTime)
            {
                PrintRealtimeState(processor.State());
                AddRealTimePlotPoint(
                    processor.State().time,
                    processor.State().latitude,
                    processor.State().longitude);
                nextConsoleOutputTime = std::floor(processor.State().time) + 1.0;
            }
        }
    }

    summary.finalState = processor.State();
    summary.success = true;
    summary.message = "Real-time loose coupling completed.";
    StopRealTimePlot();
    return true;
}
