#include "LCI_Structs.h"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>

namespace
{
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

    const Eigen::Vector3d axis = rotationVector / angle;
    return Eigen::AngleAxisd(angle, axis).toRotationMatrix();
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

void ApplyNedOffsetToPosition(
    double baseLatitude,
    double baseLongitude,
    double baseHeight,
    const Eigen::Vector3d& deltaNED,
    double& latitude,
    double& longitude,
    double& height)
{
    const double rm = MeridianRadius(baseLatitude);
    const double rn = PrimeVerticalRadius(baseLatitude);
    latitude = baseLatitude + deltaNED.x() / (rm + baseHeight);
    longitude = baseLongitude + deltaNED.y() / ((rn + baseHeight) * std::cos(baseLatitude));
    height = baseHeight - deltaNED.z();
}
}

RtkData GnssPositionToImuPosition(
    const RtkData& gnssPosition,
    const Eigen::Matrix3d& Cnb,
    const GnssLeverArm& leverArm)
{
    RtkData imuPosition = gnssPosition;
    const Eigen::Vector3d leverNED = Cnb * leverArm.BodyFRD();
    ApplyNedOffsetToPosition(
        gnssPosition.latitude,
        gnssPosition.longitude,
        gnssPosition.height,
        -leverNED,
        imuPosition.latitude,
        imuPosition.longitude,
        imuPosition.height);
    return imuPosition;
}

RtkData ImuPositionToGnssPosition(
    const RtkData& imuPosition,
    const Eigen::Matrix3d& Cnb,
    const GnssLeverArm& leverArm)
{
    RtkData gnssPosition = imuPosition;
    const Eigen::Vector3d leverNED = Cnb * leverArm.BodyFRD();
    ApplyNedOffsetToPosition(
        imuPosition.latitude,
        imuPosition.longitude,
        imuPosition.height,
        leverNED,
        gnssPosition.latitude,
        gnssPosition.longitude,
        gnssPosition.height);
    return gnssPosition;
}

std::vector<InsState> RunPureINS(
    const std::vector<ImuData>& imuData,
    const IniAlignResult& initialState,
    const PureInsConfig& config)
{
    std::vector<InsState> states;
    if (!initialState.success || imuData.empty())
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
    DcmToEulerNed(state.Cnb, state.roll, state.pitch, state.heading);
    states.push_back(state);

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

        const InsState previousState = state;
        const double latitudeForUpdate = previousState.latitude * 2.0 - midLatitude;
        const double heightForUpdate = previousState.height * 2.0 - midHeight;
        const double vNForUpdate = previousState.velocityNED.x() * 2.0 - midVelocity.x();
        const double vEForUpdate = previousState.velocityNED.y() * 2.0 - midVelocity.y();

        const Eigen::Vector3d omega_ie_n(
            omega_e_GPS * std::cos(latitudeForUpdate),
            0.0,
            -omega_e_GPS * std::sin(latitudeForUpdate));
        const Eigen::Vector3d omega_en_n(
            vEForUpdate / (PrimeVerticalRadius(latitudeForUpdate) + heightForUpdate),
            -vNForUpdate / (MeridianRadius(latitudeForUpdate) + heightForUpdate),
            -vEForUpdate * std::tan(latitudeForUpdate) /
                (PrimeVerticalRadius(latitudeForUpdate) + heightForUpdate));

        const Eigen::Vector3d correctedGyro = imu.gyro - config.gyroBias;
        const Eigen::Vector3d correctedAccel = imu.accel - config.accelBias;
        const Eigen::Vector3d deltaTheta = correctedGyro * dt;
        const Eigen::Vector3d deltaVelocity = correctedAccel * dt;

        Eigen::Vector3d scullingVelocity = deltaVelocity + 0.5 * deltaTheta.cross(deltaVelocity);
        Eigen::Vector3d coningRotation = deltaTheta;
        if (hasPreviousDelta)
        {
            scullingVelocity +=
                (previousDeltaTheta.cross(deltaVelocity) + previousDeltaVelocity.cross(deltaTheta)) / 12.0;
            coningRotation += previousDeltaTheta.cross(deltaTheta) / 12.0;
        }

        const Eigen::Vector3d zeta = (omega_ie_n + omega_en_n) * dt;
        const Eigen::Matrix3d I3 = Eigen::Matrix3d::Identity();
        const Eigen::Vector3d specificForceVelocity =
            (I3 - 0.5 * SkewSymmetric(zeta)) * previousState.Cnb * scullingVelocity;
        const Eigen::Vector3d gravityN(0.0, 0.0, NormalGravity(latitudeForUpdate, heightForUpdate));
        const Eigen::Vector3d velocityForUpdate(vNForUpdate, vEForUpdate, previousState.velocityNED.z() * 2.0 - midVelocity.z());
        const Eigen::Vector3d gravityVelocity =
            (gravityN - (2.0 * omega_ie_n + omega_en_n).cross(velocityForUpdate)) * dt;

        state.velocityNED = previousState.velocityNED + specificForceVelocity + gravityVelocity;
        midVelocity = 0.5 * (state.velocityNED + previousState.velocityNED);

        state.height = previousState.height - midVelocity.z() * dt;
        midHeight = 0.5 * (state.height + previousState.height);
        state.latitude = previousState.latitude +
            midVelocity.x() / (MeridianRadius(previousState.latitude) + midHeight) * dt;
        midLatitude = 0.5 * (state.latitude + previousState.latitude);
        state.longitude = previousState.longitude +
            midVelocity.y() / ((PrimeVerticalRadius(midLatitude) + midHeight) * std::cos(midLatitude)) * dt;

        const Eigen::Matrix3d Cbb = RotationVectorToMatrix(coningRotation);
        const Eigen::Matrix3d Cnn = RotationVectorToMatrix(-zeta);
        state.Cnb = Cnn * previousState.Cnb * Cbb;
        NormalizeDcm(state.Cnb);

        state.time = imu.time;
        DcmToEulerNed(state.Cnb, state.roll, state.pitch, state.heading);
        states.push_back(state);

        previousDeltaTheta = deltaTheta;
        previousDeltaVelocity = deltaVelocity;
        hasPreviousDelta = true;
        previousImu = imu;
    }

    return states;
}

bool SavePureINSResult(
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
