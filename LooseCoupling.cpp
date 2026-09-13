#include "LCI_Structs.h"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>

namespace
{
constexpr int kErrorStateDim = 15;

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

void DcmToEulerNed(const Eigen::Matrix3d& Cnb, double& roll, double& pitch, double& heading)
{
    roll = std::atan2(Cnb(2, 1), Cnb(2, 2));
    pitch = std::asin(std::clamp(-Cnb(2, 0), -1.0, 1.0));
    heading = std::atan2(Cnb(1, 0), Cnb(0, 0));
    if (heading < 0.0)
    {
        heading += 2.0 * M_PI;
    }
}

Eigen::Vector3d PositionInnovationNED(
    const InsState& nominalState,
    const RtkData& observedImuPosition)
{
    const double rm = MeridianRadius(nominalState.latitude);
    const double rn = PrimeVerticalRadius(nominalState.latitude);
    return Eigen::Vector3d(
        (observedImuPosition.latitude - nominalState.latitude) * (rm + nominalState.height),
        (observedImuPosition.longitude - nominalState.longitude) *
            (rn + nominalState.height) * std::cos(nominalState.latitude),
        nominalState.height - observedImuPosition.height);
}

void ApplyPositionCorrection(InsState& state, const Eigen::Vector3d& deltaPositionNED)
{
    const double rm = MeridianRadius(state.latitude);
    const double rn = PrimeVerticalRadius(state.latitude);
    state.latitude += deltaPositionNED.x() / (rm + state.height);
    state.longitude += deltaPositionNED.y() / ((rn + state.height) * std::cos(state.latitude));
    state.height -= deltaPositionNED.z();
}

void SaveEuler(InsState& state)
{
    DcmToEulerNed(state.Cnb, state.roll, state.pitch, state.heading);
}

double PositionDifferenceNormNED(const InsState& a, const InsState& b)
{
    const double rm = MeridianRadius(a.latitude);
    const double rn = PrimeVerticalRadius(a.latitude);
    const Eigen::Vector3d difference(
        (b.latitude - a.latitude) * (rm + a.height),
        (b.longitude - a.longitude) * (rn + a.height) * std::cos(a.latitude),
        a.height - b.height);
    return difference.norm();
}

double AttitudeDifferenceAngle(const InsState& a, const InsState& b)
{
    Eigen::Quaterniond qa(a.Cnb);
    Eigen::Quaterniond qb(b.Cnb);
    qa.normalize();
    qb.normalize();
    return qa.angularDistance(qb);
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

void PropagateInsState(
    const ImuData& previousImu,
    const ImuData& currentImu,
    const Eigen::Vector3d& gyroBias,
    const Eigen::Vector3d& accelBias,
    Eigen::Vector3d& previousDeltaTheta,
    Eigen::Vector3d& previousDeltaVelocity,
    bool& hasPreviousDelta,
    double& midLatitude,
    double& midHeight,
    Eigen::Vector3d& midVelocity,
    InsState& state)
{
    const double dt = currentImu.time - previousImu.time;
    if (dt <= 0.0 || dt > 1.0)
    {
        return;
    }

    const InsState previousState = state;
    const double latitudeForUpdate = previousState.latitude * 2.0 - midLatitude;
    const double heightForUpdate = previousState.height * 2.0 - midHeight;
    const double vNForUpdate = previousState.velocityNED.x() * 2.0 - midVelocity.x();
    const double vEForUpdate = previousState.velocityNED.y() * 2.0 - midVelocity.y();
    const double vDForUpdate = previousState.velocityNED.z() * 2.0 - midVelocity.z();

    const Eigen::Vector3d omegaIeN(
        omega_e_GPS * std::cos(latitudeForUpdate),
        0.0,
        -omega_e_GPS * std::sin(latitudeForUpdate));
    const Eigen::Vector3d omegaEnN(
        vEForUpdate / (PrimeVerticalRadius(latitudeForUpdate) + heightForUpdate),
        -vNForUpdate / (MeridianRadius(latitudeForUpdate) + heightForUpdate),
        -vEForUpdate * std::tan(latitudeForUpdate) /
            (PrimeVerticalRadius(latitudeForUpdate) + heightForUpdate));

    const Eigen::Vector3d deltaTheta = (currentImu.gyro - gyroBias) * dt;
    const Eigen::Vector3d deltaVelocity = (currentImu.accel - accelBias) * dt;

    Eigen::Vector3d scullingVelocity = deltaVelocity + 0.5 * deltaTheta.cross(deltaVelocity);
    Eigen::Vector3d coningRotation = deltaTheta;
    if (hasPreviousDelta)
    {
        scullingVelocity +=
            (previousDeltaTheta.cross(deltaVelocity) + previousDeltaVelocity.cross(deltaTheta)) / 12.0;
        coningRotation += previousDeltaTheta.cross(deltaTheta) / 12.0;
    }

    const Eigen::Vector3d zeta = (omegaIeN + omegaEnN) * dt;
    const Eigen::Vector3d specificForceVelocity =
        (Eigen::Matrix3d::Identity() - 0.5 * SkewSymmetric(zeta)) *
        previousState.Cnb * scullingVelocity;
    const Eigen::Vector3d gravityN(0.0, 0.0, NormalGravity(latitudeForUpdate, heightForUpdate));
    const Eigen::Vector3d velocityForUpdate(vNForUpdate, vEForUpdate, vDForUpdate);
    const Eigen::Vector3d gravityVelocity =
        (gravityN - (2.0 * omegaIeN + omegaEnN).cross(velocityForUpdate)) * dt;

    state.velocityNED = previousState.velocityNED + specificForceVelocity + gravityVelocity;
    midVelocity = 0.5 * (state.velocityNED + previousState.velocityNED);

    state.height = previousState.height - midVelocity.z() * dt;
    midHeight = 0.5 * (state.height + previousState.height);
    state.latitude = previousState.latitude +
        midVelocity.x() / (MeridianRadius(previousState.latitude) + midHeight) * dt;
    midLatitude = 0.5 * (state.latitude + previousState.latitude);
    state.longitude = previousState.longitude +
        midVelocity.y() / ((PrimeVerticalRadius(midLatitude) + midHeight) * std::cos(midLatitude)) * dt;

    state.Cnb = RotationVectorToMatrix(-zeta) * previousState.Cnb * RotationVectorToMatrix(coningRotation);
    NormalizeDcm(state.Cnb);
    state.time = currentImu.time;
    SaveEuler(state);

    previousDeltaTheta = deltaTheta;
    previousDeltaVelocity = deltaVelocity;
    hasPreviousDelta = true;
}

void PropagateCovariance(
    const InsState& state,
    const ImuData& imu,
    const Eigen::Vector3d& accelBias,
    const Eigen::Vector3d& gyroBias,
    const LooseCouplingConfig& config,
    double dt,
    Eigen::Matrix<double, kErrorStateDim, kErrorStateDim>& P)
{
    Eigen::Matrix<double, kErrorStateDim, kErrorStateDim> F =
        Eigen::Matrix<double, kErrorStateDim, kErrorStateDim>::Zero();
    F.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity();

    const Eigen::Vector3d specificForceN = state.Cnb * (imu.accel - accelBias);
    F.block<3, 3>(3, 6) = -SkewSymmetric(specificForceN);
    F.block<3, 3>(3, 12) = -state.Cnb;

    const Eigen::Vector3d omegaIeN(
        omega_e_GPS * std::cos(state.latitude),
        0.0,
        -omega_e_GPS * std::sin(state.latitude));
    F.block<3, 3>(6, 6) = -SkewSymmetric(omegaIeN);
    F.block<3, 3>(6, 9) = -state.Cnb;

    const Eigen::Matrix<double, kErrorStateDim, kErrorStateDim> Phi =
        Eigen::Matrix<double, kErrorStateDim, kErrorStateDim>::Identity() + F * dt;

    Eigen::Matrix<double, kErrorStateDim, kErrorStateDim> Q =
        Eigen::Matrix<double, kErrorStateDim, kErrorStateDim>::Zero();
    Q.block<3, 3>(3, 3) =
        Eigen::Matrix3d::Identity() * config.accelNoiseStd * config.accelNoiseStd * dt;
    Q.block<3, 3>(6, 6) =
        Eigen::Matrix3d::Identity() * config.gyroNoiseStd * config.gyroNoiseStd * dt;
    Q.block<3, 3>(9, 9) =
        Eigen::Matrix3d::Identity() * config.gyroBiasNoiseStd * config.gyroBiasNoiseStd * dt;
    Q.block<3, 3>(12, 12) =
        Eigen::Matrix3d::Identity() * config.accelBiasNoiseStd * config.accelBiasNoiseStd * dt;

    P = Phi * P * Phi.transpose() + Q;
    P = 0.5 * (P + P.transpose());
}

void GnssUpdate(
    const RtkData& rtk,
    const LooseCouplingConfig& config,
    InsState& state,
    Eigen::Vector3d& gyroBias,
    Eigen::Vector3d& accelBias,
    Eigen::Matrix<double, kErrorStateDim, kErrorStateDim>& P)
{
    const RtkData observedImuPosition = GnssPositionToImuPosition(rtk, state.Cnb, config.leverArm);
    const Eigen::Vector3d positionInnovation = PositionInnovationNED(state, observedImuPosition);
    const Eigen::Vector3d observedVelocityNED(
        rtk.velocityENU.y(),
        rtk.velocityENU.x(),
        -rtk.velocityENU.z());
    const Eigen::Vector3d velocityInnovation = observedVelocityNED - state.velocityNED;

    Eigen::Matrix<double, 6, 1> innovation;
    innovation.segment<3>(0) = positionInnovation;
    innovation.segment<3>(3) = velocityInnovation;

    Eigen::Matrix<double, 6, kErrorStateDim> H =
        Eigen::Matrix<double, 6, kErrorStateDim>::Zero();
    H.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
    H.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity();

    Eigen::Matrix<double, 6, 6> R = Eigen::Matrix<double, 6, 6>::Zero();
    R.block<3, 3>(0, 0) =
        Eigen::Matrix3d::Identity() * config.gnssPositionStd * config.gnssPositionStd;
    R.block<3, 3>(3, 3) =
        Eigen::Matrix3d::Identity() * config.gnssVelocityStd * config.gnssVelocityStd;

    KalmanUpdate<6>(innovation, H, R, state, gyroBias, accelBias, P);
}

void ZuptUpdate(
    const LooseCouplingConfig& config,
    InsState& state,
    Eigen::Vector3d& gyroBias,
    Eigen::Vector3d& accelBias,
    Eigen::Matrix<double, kErrorStateDim, kErrorStateDim>& P)
{
    Eigen::Matrix<double, 3, 1> innovation = -state.velocityNED;
    Eigen::Matrix<double, 3, kErrorStateDim> H =
        Eigen::Matrix<double, 3, kErrorStateDim>::Zero();
    H.block<3, 3>(0, 3) = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d R = Eigen::Matrix3d::Identity() *
        config.zuptVelocityStd * config.zuptVelocityStd;

    KalmanUpdate<3>(innovation, H, R, state, gyroBias, accelBias, P);
}

bool NhcUpdate(
    const LooseCouplingConfig& config,
    InsState& state,
    Eigen::Vector3d& gyroBias,
    Eigen::Vector3d& accelBias,
    Eigen::Matrix<double, kErrorStateDim, kErrorStateDim>& P)
{
    const Eigen::Matrix3d Cbn = state.Cnb.transpose();
    const Eigen::Vector3d velocityBody = Cbn * state.velocityNED;
    if (std::abs(velocityBody.y()) > config.nhcMaxLateralResidual ||
        std::abs(velocityBody.z()) > config.nhcMaxVerticalResidual)
    {
        return false;
    }

    Eigen::Matrix<double, 2, 1> innovation;
    innovation << -velocityBody.y(), -velocityBody.z();

    Eigen::Matrix<double, 2, kErrorStateDim> H =
        Eigen::Matrix<double, 2, kErrorStateDim>::Zero();
    const Eigen::Matrix3d attitudeJacobian = Cbn * SkewSymmetric(state.velocityNED);
    H.block<2, 3>(0, 3) = Cbn.block<2, 3>(1, 0);
    H.block<2, 3>(0, 6) = attitudeJacobian.block<2, 3>(1, 0);

    Eigen::Matrix2d R = Eigen::Matrix2d::Zero();
    R(0, 0) = config.nhcLateralStd * config.nhcLateralStd;
    R(1, 1) = config.nhcVerticalStd * config.nhcVerticalStd;

    KalmanUpdate<2>(innovation, H, R, state, gyroBias, accelBias, P);
    return true;
}
}

std::vector<InsState> RunLooseCoupled(
    const std::vector<ImuData>& imuData,
    const std::vector<RtkData>& rtkData,
    const IniAlignResult& initialState,
    const LooseCouplingConfig& config)
{
    std::vector<InsState> states;
    if (!initialState.success || imuData.empty() || rtkData.empty())
    {
        return states;
    }

    InsState state;
    state.time = config.useStartTime ? config.startTime : initialState.endTime;
    state.latitude = initialState.latitude;
    state.longitude = initialState.longitude;
    state.height = initialState.height;
    state.velocityNED = initialState.velocityNED;
    state.Cnb = initialState.Cnb;
    SaveEuler(state);

    Eigen::Vector3d gyroBias = initialState.gyroBias;
    Eigen::Vector3d accelBias = initialState.accelBias;

    Eigen::Matrix<double, kErrorStateDim, kErrorStateDim> P =
        Eigen::Matrix<double, kErrorStateDim, kErrorStateDim>::Zero();
    P.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity() * 10.0;
    P.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity() * 1.0;
    P.block<3, 3>(6, 6) = Eigen::Matrix3d::Identity() * std::pow(2.0 * DegToRad, 2);
    P.block<3, 3>(9, 9) = Eigen::Matrix3d::Identity() * std::pow(0.05 * DegToRad, 2);
    P.block<3, 3>(12, 12) = Eigen::Matrix3d::Identity() * 0.01;

    ImuData previousImu;
    bool hasPreviousImu = false;
    for (const ImuData& imu : imuData)
    {
        if (imu.time <= state.time)
        {
            previousImu = imu;
            hasPreviousImu = true;
            continue;
        }
        break;
    }

    if (!hasPreviousImu)
    {
        return states;
    }

    Eigen::Vector3d previousDeltaTheta = Eigen::Vector3d::Zero();
    Eigen::Vector3d previousDeltaVelocity = Eigen::Vector3d::Zero();
    bool hasPreviousDelta = false;
    double midLatitude = state.latitude;
    double midHeight = state.height;
    Eigen::Vector3d midVelocity = state.velocityNED;
    std::size_t nextRtkIndex = 0;
    while (nextRtkIndex < rtkData.size() && rtkData[nextRtkIndex].time <= state.time)
    {
        ++nextRtkIndex;
    }

    states.push_back(state);
    double lastNhcTime = state.time;
    for (const ImuData& imu : imuData)
    {
        if (imu.time <= state.time)
        {
            continue;
        }
        if (config.useEndTime && imu.time > config.endTime)
        {
            break;
        }

        const double dt = imu.time - previousImu.time;
        if (dt <= 0.0 || dt > 1.0)
        {
            previousImu = imu;
            continue;
        }

        PropagateInsState(
            previousImu,
            imu,
            gyroBias,
            accelBias,
            previousDeltaTheta,
            previousDeltaVelocity,
            hasPreviousDelta,
            midLatitude,
            midHeight,
            midVelocity,
            state);
        PropagateCovariance(state, imu, accelBias, gyroBias, config, dt, P);

        if (config.useNHC &&
            state.velocityNED.norm() >= config.nhcMinSpeed &&
            state.time - lastNhcTime >= config.nhcInterval)
        {
            if (NhcUpdate(config, state, gyroBias, accelBias, P))
            {
                lastNhcTime = state.time;
            }
        }

        while (nextRtkIndex < rtkData.size() && rtkData[nextRtkIndex].time <= state.time)
        {
            GnssUpdate(rtkData[nextRtkIndex], config, state, gyroBias, accelBias, P);
            const Eigen::Vector3d observedVelocityNED(
                rtkData[nextRtkIndex].velocityENU.y(),
                rtkData[nextRtkIndex].velocityENU.x(),
                -rtkData[nextRtkIndex].velocityENU.z());
            if (config.useZUPT && observedVelocityNED.norm() <= config.zuptSpeedThreshold)
            {
                ZuptUpdate(config, state, gyroBias, accelBias, P);
            }
            ++nextRtkIndex;
        }

        states.push_back(state);
        previousImu = imu;
    }

    return states;
}

std::vector<InsState> RunLooseCoupledBackward(
    const std::vector<ImuData>& imuData,
    const std::vector<RtkData>& rtkData,
    const IniAlignResult& initialBias,
    const InsState& endState,
    const LooseCouplingConfig& config)
{
    if (imuData.empty() || rtkData.empty())
    {
        return {};
    }

    const double originalStartTime = config.useStartTime ? config.startTime : initialBias.endTime;
    const double originalEndTime = config.useEndTime ? config.endTime : imuData.back().time;

    std::vector<ImuData> reversedImu;
    reversedImu.reserve(imuData.size());
    for (auto it = imuData.rbegin(); it != imuData.rend(); ++it)
    {
        if (it->time < originalStartTime || it->time > originalEndTime)
        {
            continue;
        }

        ImuData reversed;
        reversed.time = originalStartTime + originalEndTime - it->time;
        reversed.gyro = -it->gyro;
        reversed.accel = -it->accel;
        reversedImu.push_back(reversed);
    }
    std::vector<RtkData> reversedRtk;
    reversedRtk.reserve(rtkData.size());
    for (auto it = rtkData.rbegin(); it != rtkData.rend(); ++it)
    {
        if (it->time < originalStartTime || it->time > originalEndTime)
        {
            continue;
        }

        RtkData reversed = *it;
        reversed.time = originalStartTime + originalEndTime - it->time;
        reversed.velocityENU = -it->velocityENU;
        reversedRtk.push_back(reversed);
    }
    IniAlignResult reverseInitial;
    reverseInitial.success = true;
    reverseInitial.endTime = originalStartTime;
    reverseInitial.latitude = endState.latitude;
    reverseInitial.longitude = endState.longitude;
    reverseInitial.height = endState.height;
    reverseInitial.velocityNED = -endState.velocityNED;
    reverseInitial.Cnb = endState.Cnb;
    reverseInitial.gyroBias = initialBias.gyroBias;
    reverseInitial.accelBias = initialBias.accelBias;

    LooseCouplingConfig reverseConfig = config;
    reverseConfig.startTime = originalStartTime;
    reverseConfig.endTime = originalEndTime;
    reverseConfig.useStartTime = true;
    reverseConfig.useEndTime = config.useEndTime;

    std::vector<InsState> reversedStates =
        RunLooseCoupled(reversedImu, reversedRtk, reverseInitial, reverseConfig);

    std::vector<InsState> states;
    states.reserve(reversedStates.size());
    for (auto it = reversedStates.rbegin(); it != reversedStates.rend(); ++it)
    {
        InsState state = *it;
        state.time = originalStartTime + originalEndTime - it->time;
        state.velocityNED = -it->velocityNED;
        states.push_back(state);
    }

    return states;
}

bool SaveLooseCoupledResult(
    const std::vector<InsState>& states,
    const std::string& filePath)
{
    namespace fs = std::filesystem;

    const fs::path outputPath(filePath);
    if (outputPath.has_parent_path())
    {
        fs::create_directories(outputPath.parent_path());
    }

    std::ofstream output(filePath);
    if (!output.is_open())
    {
        return false;
    }

    output << "Time(s) Latitude(deg) Longitude(deg) Height(m) "
           << "VN(m/s) VE(m/s) VD(m/s) Roll(deg) Pitch(deg) Heading(deg)\n";

    output << std::fixed << std::setprecision(10);
    for (const InsState& state : states)
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

    return true;
}

std::vector<InsState> FuseForwardBackward(
    const std::vector<InsState>& forwardStates,
    const std::vector<InsState>& backwardStates,
    double forwardWeight)
{
    std::vector<InsState> smoothedStates;
    if (forwardStates.empty() || backwardStates.empty())
    {
        return smoothedStates;
    }

    const double wForward = std::clamp(forwardWeight, 0.0, 1.0);
    const double wBackward = 1.0 - wForward;
    constexpr double kTimeMatchTolerance = 0.01;
    constexpr double kMaxFusePositionDifference = 5.0;
    constexpr double kMaxFuseVelocityDifference = 2.0;
    constexpr double kMaxFuseAttitudeDifference = 10.0 * DegToRad;
    std::size_t backwardIndex = 0;
    smoothedStates.reserve(forwardStates.size());

    for (const InsState& forward : forwardStates)
    {
        while (backwardIndex + 1 < backwardStates.size() &&
               backwardStates[backwardIndex + 1].time < forward.time)
        {
            ++backwardIndex;
        }

        const InsState* backward = nullptr;
        if (backwardIndex < backwardStates.size() &&
            std::abs(backwardStates[backwardIndex].time - forward.time) <= kTimeMatchTolerance)
        {
            backward = &backwardStates[backwardIndex];
        }
        if (backwardIndex + 1 < backwardStates.size() &&
            std::abs(backwardStates[backwardIndex + 1].time - forward.time) <= kTimeMatchTolerance)
        {
            const double currentDiff = backward == nullptr
                ? std::numeric_limits<double>::max()
                : std::abs(backward->time - forward.time);
            const double nextDiff = std::abs(backwardStates[backwardIndex + 1].time - forward.time);
            if (nextDiff < currentDiff)
            {
                backward = &backwardStates[backwardIndex + 1];
            }
        }

        if (backward == nullptr ||
            PositionDifferenceNormNED(forward, *backward) > kMaxFusePositionDifference ||
            (forward.velocityNED - backward->velocityNED).norm() > kMaxFuseVelocityDifference ||
            AttitudeDifferenceAngle(forward, *backward) > kMaxFuseAttitudeDifference)
        {
            smoothedStates.push_back(forward);
            continue;
        }

        InsState smoothed;
        smoothed.time = forward.time;
        smoothed.latitude = wForward * forward.latitude + wBackward * backward->latitude;
        smoothed.longitude = wForward * forward.longitude + wBackward * backward->longitude;
        smoothed.height = wForward * forward.height + wBackward * backward->height;
        smoothed.velocityNED = wForward * forward.velocityNED + wBackward * backward->velocityNED;

        Eigen::Quaterniond qForward(forward.Cnb);
        Eigen::Quaterniond qBackward(backward->Cnb);
        if (qForward.dot(qBackward) < 0.0)
        {
            qBackward.coeffs() *= -1.0;
        }
        Eigen::Quaterniond qSmoothed = qForward.slerp(wBackward, qBackward);
        qSmoothed.normalize();
        smoothed.Cnb = qSmoothed.toRotationMatrix();
        NormalizeDcm(smoothed.Cnb);
        SaveEuler(smoothed);

        smoothedStates.push_back(smoothed);
    }

    return smoothedStates;
}
