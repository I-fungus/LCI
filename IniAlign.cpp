#include "LCI_Structs.h"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <sstream>

namespace
{
double NormalizeAngle(double angle)
{
    while (angle < 0.0)
    {
        angle += 2.0 * M_PI;
    }
    while (angle >= 2.0 * M_PI)
    {
        angle -= 2.0 * M_PI;
    }
    return angle;
}

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

Eigen::Matrix3d EulerNedToDcm(double roll, double pitch, double heading)
{
    const double sr = std::sin(roll);
    const double cr = std::cos(roll);
    const double sp = std::sin(pitch);
    const double cp = std::cos(pitch);
    const double sh = std::sin(heading);
    const double ch = std::cos(heading);

    Eigen::Matrix3d Cnb;
    Cnb << cp * ch, sr * sp * ch - cr * sh, cr * sp * ch + sr * sh,
           cp * sh, sr * sp * sh + cr * ch, cr * sp * sh - sr * ch,
           -sp, sr * cp, cr * cp;
    return Cnb;
}

void DcmToEulerNed(const Eigen::Matrix3d& Cnb, double& roll, double& pitch, double& heading)
{
    roll = std::atan2(Cnb(2, 1), Cnb(2, 2));
    pitch = std::asin(std::clamp(-Cnb(2, 0), -1.0, 1.0));
    heading = NormalizeAngle(std::atan2(Cnb(1, 0), Cnb(0, 0)));
}
}

std::string ResolveProjectDataPath(const std::string& fileName)
{
    namespace fs = std::filesystem;

    fs::path current = fs::current_path();
    for (int i = 0; i < 6; ++i)
    {
        const fs::path candidate = current / "data" / fileName;
        if (fs::exists(candidate))
        {
            return candidate.string();
        }

        if (!current.has_parent_path() || current == current.parent_path())
        {
            break;
        }
        current = current.parent_path();
    }

    return (fs::current_path() / "data" / fileName).string();
}

double NormalGravity(double latitude, double height)
{
    const double sinLat = std::sin(latitude);
    const double sinLat2 = sinLat * sinLat;
    const double sinLat4 = sinLat2 * sinLat2;
    const double g0 = 9.7803267715 * (1.0 + 0.0052790414 * sinLat2 + 0.0000232718 * sinLat4);
    return g0 - (3.087691089e-6 - 4.397731e-9 * sinLat2) * height + 0.721e-12 * height * height;
}

std::vector<ImuData> LoadImuData(const std::string& filePath)
{
    std::ifstream input(filePath);
    std::vector<ImuData> data;
    if (!input.is_open())
    {
        return data;
    }

    std::string line;
    std::vector<double> values;
    while (std::getline(input, line))
    {
        if (!ParseNumericLine(line, values) || values.size() < 7)
        {
            continue;
        }

        ImuData imu;
        imu.time = values[0];
        imu.gyro = Eigen::Vector3d(values[1], values[2], values[3]) * DegToRad;
        imu.accel = Eigen::Vector3d(values[4], values[5], values[6]);
        data.push_back(imu);
    }

    return data;
}

std::vector<RtkData> LoadRtkData(const std::string& filePath)
{
    std::ifstream input(filePath);
    std::vector<RtkData> data;
    if (!input.is_open())
    {
        return data;
    }

    std::string line;
    std::vector<double> values;
    while (std::getline(input, line))
    {
        if (!ParseNumericLine(line, values) || values.size() < 14)
        {
            continue;
        }

        RtkData rtk;
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
        data.push_back(rtk);
    }

    return data;
}

bool AverageRtkData(
    const std::vector<RtkData>& rtkData,
    double startTime,
    double endTime,
    RtkData& averageRtk,
    std::size_t& sampleCount)
{
    averageRtk = RtkData();
    sampleCount = 0;

    if (rtkData.empty() || endTime < startTime)
    {
        return false;
    }

    double headingSinSum = 0.0;
    double headingCosSum = 0.0;
    double pitchSum = 0.0;
    double rollSum = 0.0;
    double qualitySum = 0.0;

    for (const RtkData& rtk : rtkData)
    {
        if (rtk.time < startTime)
        {
            continue;
        }
        if (rtk.time > endTime)
        {
            break;
        }

        averageRtk.time += rtk.time;
        averageRtk.gridY += rtk.gridY;
        averageRtk.gridX += rtk.gridX;
        averageRtk.gridZ += rtk.gridZ;
        averageRtk.height += rtk.height;
        averageRtk.latitude += rtk.latitude;
        averageRtk.longitude += rtk.longitude;
        averageRtk.velocityENU += rtk.velocityENU;
        headingSinSum += std::sin(rtk.heading);
        headingCosSum += std::cos(rtk.heading);
        pitchSum += rtk.pitch;
        rollSum += rtk.roll;
        qualitySum += static_cast<double>(rtk.quality);
        ++sampleCount;
    }

    if (sampleCount == 0)
    {
        return false;
    }

    const double invCount = 1.0 / static_cast<double>(sampleCount);
    averageRtk.time *= invCount;
    averageRtk.gridY *= invCount;
    averageRtk.gridX *= invCount;
    averageRtk.gridZ *= invCount;
    averageRtk.height *= invCount;
    averageRtk.latitude *= invCount;
    averageRtk.longitude *= invCount;
    averageRtk.velocityENU *= invCount;
    averageRtk.heading = NormalizeAngle(std::atan2(headingSinSum, headingCosSum));
    averageRtk.pitch = pitchSum * invCount;
    averageRtk.roll = rollSum * invCount;
    averageRtk.quality = static_cast<int>(std::round(qualitySum * invCount));

    return true;
}

IniAlignResult InitialAlignment(
    const std::vector<ImuData>& imuData,
    const RtkData& initRtk,
    const IniAlignConfig& config)
{
    IniAlignResult result;
    result.latitude = initRtk.latitude;
    result.longitude = initRtk.longitude;
    result.height = initRtk.height;
    result.velocityNED = Eigen::Vector3d(
        initRtk.velocityENU.y(),
        initRtk.velocityENU.x(),
        -initRtk.velocityENU.z());

    if (imuData.empty())
    {
        result.message = "IMU data is empty.";
        return result;
    }

    const double startTime = config.useStartTime ? config.startTime : imuData.front().time;
    const double endTime = startTime + std::max(config.duration, 0.0);
    Eigen::Vector3d gyroSum = Eigen::Vector3d::Zero();
    Eigen::Vector3d accelSum = Eigen::Vector3d::Zero();

    bool hasFirst = false;
    for (const ImuData& imu : imuData)
    {
        if (imu.time < startTime)
        {
            continue;
        }
        if (imu.time > endTime)
        {
            break;
        }

        if (!hasFirst)
        {
            result.startTime = imu.time;
            hasFirst = true;
        }
        result.endTime = imu.time;
        gyroSum += imu.gyro;
        accelSum += imu.accel;
        ++result.sampleCount;
    }

    if (result.sampleCount < 20)
    {
        result.message = "Not enough static IMU samples for initial alignment.";
        return result;
    }

    result.meanGyro = gyroSum / static_cast<double>(result.sampleCount);
    result.meanAccel = accelSum / static_cast<double>(result.sampleCount);

    const Eigen::Vector3d gravityBody = -result.meanAccel;
    if (gravityBody.norm() < 1.0)
    {
        result.message = "Mean accelerometer norm is invalid.";
        return result;
    }

    const Eigen::Vector3d gravityUnitBody = gravityBody.normalized();
    result.roll = std::atan2(gravityUnitBody.y(), gravityUnitBody.z());
    result.pitch = std::asin(std::clamp(-gravityUnitBody.x(), -1.0, 1.0));
    result.heading = NormalizeAngle(config.initialHeading);
    result.Cnb = EulerNedToDcm(result.roll, result.pitch, result.heading);

    const RtkData imuPosition = GnssPositionToImuPosition(initRtk, result.Cnb);
    result.latitude = imuPosition.latitude;
    result.longitude = imuPosition.longitude;
    result.height = imuPosition.height;

    const Eigen::Vector3d normalGravityNav(0.0, 0.0, NormalGravity(result.latitude, result.height));
    const Eigen::Vector3d specificForceBody = result.Cnb.transpose() * (-normalGravityNav);
    result.gyroBias = result.meanGyro;
    result.accelBias = result.meanAccel - specificForceBody;

    result.success = true;
    result.message = "Initial alignment completed.";
    return result;
}
