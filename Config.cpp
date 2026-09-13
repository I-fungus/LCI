#include "LCI_Structs.h"

#include <algorithm>
#include <cctype>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <unordered_map>

namespace
{
std::string Trim(const std::string& text)
{
    const auto begin = std::find_if_not(text.begin(), text.end(), [](unsigned char c) {
        return std::isspace(c) != 0;
    });
    const auto end = std::find_if_not(text.rbegin(), text.rend(), [](unsigned char c) {
        return std::isspace(c) != 0;
    }).base();

    if (begin >= end)
    {
        return {};
    }
    return std::string(begin, end);
}

std::string ToLower(std::string text)
{
    std::transform(text.begin(), text.end(), text.begin(), [](unsigned char c) {
        return static_cast<char>(std::tolower(c));
    });
    return text;
}

bool ParseBool(const std::string& value, bool defaultValue)
{
    const std::string lower = ToLower(Trim(value));
    if (lower == "1" || lower == "true" || lower == "yes" || lower == "on")
    {
        return true;
    }
    if (lower == "0" || lower == "false" || lower == "no" || lower == "off")
    {
        return false;
    }
    return defaultValue;
}

double ParseDouble(const std::string& value, double defaultValue)
{
    try
    {
        return std::stod(value);
    }
    catch (...)
    {
        return defaultValue;
    }
}

void EnsureDefaultConfig(const std::string& filePath)
{
    namespace fs = std::filesystem;
    const fs::path path(filePath);
    if (fs::exists(path))
    {
        return;
    }

    if (path.has_parent_path())
    {
        fs::create_directories(path.parent_path());
    }

    std::ofstream output(filePath);
    output << "# LCI configuration\n"
           << "processing_mode=offline\n"
           << "imu_input=imu_data.txt\n"
           << "rtk_input=rtk_result.pos\n"
           << "run_pure_ins=true\n"
           << "run_forward_filter=true\n"
           << "run_backward_filter=true\n"
           << "run_forward_backward_smoothing=true\n"
           << "align_duration=270.0\n"
           << "initial_heading_deg=267.0\n"
           << "pure_ins_output=output/pure_ins_result.nav\n"
           << "forward_output=output/loose_coupled_forward.nav\n"
           << "backward_output=output/loose_coupled_backward.nav\n"
           << "smoothed_output=output/loose_coupled_smoothed.nav\n"
           << "realtime_output=output/loose_coupled_realtime.nav\n"
           << "gnss_position_std=0.5\n"
           << "gnss_velocity_std=0.05\n"
           << "gyro_noise_std_deg=0.02\n"
           << "accel_noise_std=0.05\n"
           << "gyro_bias_noise_std=1.0e-6\n"
           << "accel_bias_noise_std=1.0e-4\n"
           << "use_zupt=true\n"
           << "use_nhc=true\n"
           << "zupt_speed_threshold=0.05\n"
           << "zupt_velocity_std=0.02\n"
           << "nhc_min_speed=0.5\n"
           << "nhc_interval=0.1\n"
           << "nhc_lateral_std=0.05\n"
           << "nhc_vertical_std=0.05\n"
           << "nhc_max_lateral_residual=1.5\n"
           << "nhc_max_vertical_residual=1.5\n"
           << "lever_arm_right=0.17\n"
           << "lever_arm_forward=0.065\n"
           << "lever_arm_up=0.0\n";
}
}

AppConfig LoadAppConfig(const std::string& filePath)
{
    EnsureDefaultConfig(filePath);

    AppConfig config;
    std::ifstream input(filePath);
    std::unordered_map<std::string, std::string> values;
    std::string line;
    while (std::getline(input, line))
    {
        const std::size_t comment = line.find('#');
        if (comment != std::string::npos)
        {
            line = line.substr(0, comment);
        }

        const std::size_t eq = line.find('=');
        if (eq == std::string::npos)
        {
            continue;
        }

        const std::string key = ToLower(Trim(line.substr(0, eq)));
        const std::string value = Trim(line.substr(eq + 1));
        if (!key.empty())
        {
            values[key] = value;
        }
    }

    auto get = [&values](const std::string& key) -> std::string {
        const auto it = values.find(key);
        return it == values.end() ? std::string() : it->second;
    };

    const std::string processingMode = ToLower(get("processing_mode"));
    config.useRealTimeMode =
        processingMode == "realtime" || processingMode == "real_time" || processingMode == "rt";
    config.imuInput = get("imu_input").empty() ? config.imuInput : get("imu_input");
    config.rtkInput = get("rtk_input").empty() ? config.rtkInput : get("rtk_input");
    config.runPureINS = ParseBool(get("run_pure_ins"), config.runPureINS);
    config.runForwardFilter = ParseBool(get("run_forward_filter"), config.runForwardFilter);
    config.runBackwardFilter = ParseBool(get("run_backward_filter"), config.runBackwardFilter);
    config.runForwardBackwardSmoothing =
        ParseBool(get("run_forward_backward_smoothing"), config.runForwardBackwardSmoothing);
    config.pureInsOutput = get("pure_ins_output").empty() ? config.pureInsOutput : get("pure_ins_output");
    config.forwardOutput = get("forward_output").empty() ? config.forwardOutput : get("forward_output");
    config.backwardOutput = get("backward_output").empty() ? config.backwardOutput : get("backward_output");
    config.smoothedOutput = get("smoothed_output").empty() ? config.smoothedOutput : get("smoothed_output");
    config.realTimeOutput = get("realtime_output").empty() ? config.realTimeOutput : get("realtime_output");
    config.alignConfig.duration = ParseDouble(get("align_duration"), config.alignConfig.duration);
    config.alignConfig.initialHeading =
        ParseDouble(get("initial_heading_deg"), config.alignConfig.initialHeading * RadToDeg) * DegToRad;

    config.looseConfig.gnssPositionStd =
        ParseDouble(get("gnss_position_std"), config.looseConfig.gnssPositionStd);
    config.looseConfig.gnssVelocityStd =
        ParseDouble(get("gnss_velocity_std"), config.looseConfig.gnssVelocityStd);
    config.looseConfig.gyroNoiseStd =
        ParseDouble(get("gyro_noise_std_deg"), config.looseConfig.gyroNoiseStd * RadToDeg) * DegToRad;
    config.looseConfig.accelNoiseStd =
        ParseDouble(get("accel_noise_std"), config.looseConfig.accelNoiseStd);
    config.looseConfig.gyroBiasNoiseStd =
        ParseDouble(get("gyro_bias_noise_std"), config.looseConfig.gyroBiasNoiseStd);
    config.looseConfig.accelBiasNoiseStd =
        ParseDouble(get("accel_bias_noise_std"), config.looseConfig.accelBiasNoiseStd);
    config.looseConfig.useZUPT = ParseBool(get("use_zupt"), config.looseConfig.useZUPT);
    config.looseConfig.useNHC = ParseBool(get("use_nhc"), config.looseConfig.useNHC);
    config.looseConfig.zuptSpeedThreshold =
        ParseDouble(get("zupt_speed_threshold"), config.looseConfig.zuptSpeedThreshold);
    config.looseConfig.zuptVelocityStd =
        ParseDouble(get("zupt_velocity_std"), config.looseConfig.zuptVelocityStd);
    config.looseConfig.nhcMinSpeed =
        ParseDouble(get("nhc_min_speed"), config.looseConfig.nhcMinSpeed);
    config.looseConfig.nhcInterval =
        ParseDouble(get("nhc_interval"), config.looseConfig.nhcInterval);
    config.looseConfig.nhcLateralStd =
        ParseDouble(get("nhc_lateral_std"), config.looseConfig.nhcLateralStd);
    config.looseConfig.nhcVerticalStd =
        ParseDouble(get("nhc_vertical_std"), config.looseConfig.nhcVerticalStd);
    config.looseConfig.nhcMaxLateralResidual =
        ParseDouble(get("nhc_max_lateral_residual"), config.looseConfig.nhcMaxLateralResidual);
    config.looseConfig.nhcMaxVerticalResidual =
        ParseDouble(get("nhc_max_vertical_residual"), config.looseConfig.nhcMaxVerticalResidual);
    config.looseConfig.leverArm.right =
        ParseDouble(get("lever_arm_right"), config.looseConfig.leverArm.right);
    config.looseConfig.leverArm.forward =
        ParseDouble(get("lever_arm_forward"), config.looseConfig.leverArm.forward);
    config.looseConfig.leverArm.up =
        ParseDouble(get("lever_arm_up"), config.looseConfig.leverArm.up);

    return config;
}
