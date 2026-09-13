#include "LCI_Structs.h"

#include <filesystem>
#include <iomanip>
#include <iostream>

int main()
{
    const AppConfig appConfig = LoadAppConfig("config/lci_config.ini");
    const std::string imuPath = ResolveProjectDataPath(appConfig.imuInput);
    const std::string rtkPath = ResolveProjectDataPath(appConfig.rtkInput);

    if (appConfig.useRealTimeMode)
    {
        std::cout << "Processing mode: realtime file playback\n";
        std::cout << "Streaming IMU: " << imuPath << '\n';
        std::cout << "Streaming RTK: " << rtkPath << '\n';

        RealTimeProcessSummary summary;
        if (!RunRealTimeLooseCoupling(imuPath, rtkPath, appConfig, summary))
        {
            std::cerr << "Real-time loose coupling failed: " << summary.message << '\n';
            return 1;
        }

        std::cout << std::fixed << std::setprecision(10);
        std::cout << "\n" << summary.message << '\n';
        std::cout << "Alignment samples: " << summary.alignment.sampleCount
                  << "  Time: " << summary.alignment.startTime
                  << " - " << summary.alignment.endTime << " s\n";
        std::cout << "Streamed IMU lines: " << summary.imuLines << '\n';
        std::cout << "Streamed RTK lines: " << summary.rtkLines << '\n';
        std::cout << "Output states: " << summary.outputStates << '\n';
        std::cout << "GNSS updates: " << summary.gnssUpdates << '\n';
        std::cout << "ZUPT updates: " << summary.zuptUpdates << '\n';
        std::cout << "NHC updates: " << summary.nhcUpdates << '\n';
        std::cout << "Real-time loose coupled result saved: "
                  << appConfig.realTimeOutput << '\n';
        std::cout << "Final realtime LC time: " << summary.finalState.time << " s\n";
        std::cout << "Final realtime LC position: lat "
                  << summary.finalState.latitude * RadToDeg
                  << " deg, lon " << summary.finalState.longitude * RadToDeg
                  << " deg, h " << summary.finalState.height << " m\n";
        std::cout << "Final realtime LC velocity NED (m/s): "
                  << summary.finalState.velocityNED.transpose() << '\n';
        std::cout << "Final realtime LC attitude (deg): "
                  << "heading " << summary.finalState.heading * RadToDeg
                  << ", pitch " << summary.finalState.pitch * RadToDeg
                  << ", roll " << summary.finalState.roll * RadToDeg << '\n';
        return 0;
    }

    std::cout << "Loading IMU: " << imuPath << '\n';
    const std::vector<ImuData> imuData = LoadImuData(imuPath);
    std::cout << "Loading RTK: " << rtkPath << '\n';
    const std::vector<RtkData> rtkData = LoadRtkData(rtkPath);

    if (imuData.empty())
    {
        std::cerr << "Failed to load IMU data.\n";
        return 1;
    }
    if (rtkData.empty())
    {
        std::cerr << "Failed to load RTK data.\n";
        return 1;
    }

    IniAlignConfig alignConfig = appConfig.alignConfig;
    alignConfig.startTime = rtkData.front().time;
    alignConfig.useStartTime = true;

    RtkData initRtk;
    std::size_t rtkAverageCount = 0;
    const double alignEndTime = alignConfig.startTime + alignConfig.duration;
    if (!AverageRtkData(rtkData, alignConfig.startTime, alignEndTime, initRtk, rtkAverageCount))
    {
        std::cerr << "Failed to average RTK data for initial position.\n";
        return 1;
    }

    const IniAlignResult align = InitialAlignment(imuData, initRtk, alignConfig);
    if (!align.success)
    {
        std::cerr << "Initial alignment failed: " << align.message << '\n';
        return 1;
    }

    std::cout << std::fixed << std::setprecision(10);
    std::cout << "\n" << align.message << '\n';
    std::cout << "Samples: " << align.sampleCount
              << "  Time: " << align.startTime << " - " << align.endTime << " s\n";
    std::cout << "RTK average samples: " << rtkAverageCount
              << "  Time: " << alignConfig.startTime << " - " << alignEndTime << " s\n";
    std::cout << "Position: lat " << align.latitude * RadToDeg
              << " deg, lon " << align.longitude * RadToDeg
              << " deg, h " << align.height << " m\n";
    std::cout << "Mean gyro (deg/s): "
              << align.meanGyro.transpose() * RadToDeg << '\n';
    std::cout << "Mean accel (m/s^2): "
              << align.meanAccel.transpose() << '\n';
    std::cout << "Estimated gyro bias (deg/s): "
              << align.gyroBias.transpose() * RadToDeg << '\n';
    std::cout << "Estimated accel bias (m/s^2): "
              << align.accelBias.transpose() << '\n';
    std::cout << "Initial attitude (deg): "
              << "heading " << align.heading * RadToDeg
              << ", pitch " << align.pitch * RadToDeg
              << ", roll " << align.roll * RadToDeg << '\n';
    std::cout << "Cnb (body to NED):\n" << align.Cnb << '\n';

    if (appConfig.runPureINS)
    {
        PureInsConfig pureInsConfig;
        pureInsConfig.startTime = align.endTime;
        pureInsConfig.useStartTime = true;
        pureInsConfig.gyroBias = align.gyroBias;
        pureInsConfig.accelBias = align.accelBias;
        const std::vector<InsState> pureInsStates = RunPureINS(imuData, align, pureInsConfig);
        std::cout << "\nPure INS states: " << pureInsStates.size() << '\n';
        if (!SavePureINSResult(pureInsStates, appConfig.pureInsOutput))
        {
            std::cerr << "Failed to save Pure INS result: " << appConfig.pureInsOutput << '\n';
            return 1;
        }
        std::cout << "Pure INS result saved: " << appConfig.pureInsOutput << '\n';
        if (!pureInsStates.empty())
        {
            const InsState& finalState = pureInsStates.back();
            std::cout << "Final INS time: " << finalState.time << " s\n";
            std::cout << "Final INS position: lat " << finalState.latitude * RadToDeg
                      << " deg, lon " << finalState.longitude * RadToDeg
                      << " deg, h " << finalState.height << " m\n";
            std::cout << "Final INS velocity NED (m/s): "
                      << finalState.velocityNED.transpose() << '\n';
            std::cout << "Final INS attitude (deg): "
                      << "heading " << finalState.heading * RadToDeg
                      << ", pitch " << finalState.pitch * RadToDeg
                      << ", roll " << finalState.roll * RadToDeg << '\n';
        }
    }

    LooseCouplingConfig lcConfig = appConfig.looseConfig;
    lcConfig.startTime = align.endTime;
    lcConfig.useStartTime = true;
    std::vector<InsState> forwardStates;
    std::vector<InsState> backwardStates;
    if (appConfig.runForwardFilter)
    {
        forwardStates = RunLooseCoupled(imuData, rtkData, align, lcConfig);
        std::cout << "\nForward loose coupled states: " << forwardStates.size() << '\n';
        if (!SaveLooseCoupledResult(forwardStates, appConfig.forwardOutput))
        {
            std::cerr << "Failed to save forward loose coupled result: " << appConfig.forwardOutput << '\n';
            return 1;
        }
        std::cout << "Forward loose coupled result saved: " << appConfig.forwardOutput << '\n';
        if (!forwardStates.empty())
        {
            const InsState& finalState = forwardStates.back();
            std::cout << "Final forward LC time: " << finalState.time << " s\n";
            std::cout << "Final forward LC position: lat " << finalState.latitude * RadToDeg
                      << " deg, lon " << finalState.longitude * RadToDeg
                      << " deg, h " << finalState.height << " m\n";
            std::cout << "Final forward LC velocity NED (m/s): "
                      << finalState.velocityNED.transpose() << '\n';
            std::cout << "Final forward LC attitude (deg): "
                      << "heading " << finalState.heading * RadToDeg
                      << ", pitch " << finalState.pitch * RadToDeg
                      << ", roll " << finalState.roll * RadToDeg << '\n';
        }
    }

    if (appConfig.runBackwardFilter)
    {
        InsState backwardEndState;
        if (!forwardStates.empty())
        {
            backwardEndState = forwardStates.back();
        }
        else
        {
            backwardEndState.time = imuData.back().time;
            backwardEndState.latitude = align.latitude;
            backwardEndState.longitude = align.longitude;
            backwardEndState.height = align.height;
            backwardEndState.velocityNED = align.velocityNED;
            backwardEndState.Cnb = align.Cnb;
            backwardEndState.roll = align.roll;
            backwardEndState.pitch = align.pitch;
            backwardEndState.heading = align.heading;
        }

        backwardStates =
            RunLooseCoupledBackward(imuData, rtkData, align, backwardEndState, lcConfig);
        std::cout << "\nBackward loose coupled states: " << backwardStates.size() << '\n';
        if (!SaveLooseCoupledResult(backwardStates, appConfig.backwardOutput))
        {
            std::cerr << "Failed to save backward loose coupled result: " << appConfig.backwardOutput << '\n';
            return 1;
        }
        std::cout << "Backward loose coupled result saved: " << appConfig.backwardOutput << '\n';
        if (!backwardStates.empty())
        {
            const InsState& finalState = backwardStates.back();
            std::cout << "Final backward LC time: " << finalState.time << " s\n";
            std::cout << "Final backward LC position: lat " << finalState.latitude * RadToDeg
                      << " deg, lon " << finalState.longitude * RadToDeg
                      << " deg, h " << finalState.height << " m\n";
            std::cout << "Final backward LC velocity NED (m/s): "
                      << finalState.velocityNED.transpose() << '\n';
            std::cout << "Final backward LC attitude (deg): "
                      << "heading " << finalState.heading * RadToDeg
                      << ", pitch " << finalState.pitch * RadToDeg
                      << ", roll " << finalState.roll * RadToDeg << '\n';
        }
    }

    if (appConfig.runForwardBackwardSmoothing)
    {
        if (forwardStates.empty() || backwardStates.empty())
        {
            std::cerr << "\nForward-backward smoothing skipped: forward or backward states are empty.\n";
        }
        else
        {
            const std::vector<InsState> smoothedStates =
                FuseForwardBackward(forwardStates, backwardStates);
            std::cout << "\nForward-backward smoothed states: " << smoothedStates.size() << '\n';
            if (!SaveLooseCoupledResult(smoothedStates, appConfig.smoothedOutput))
            {
                std::cerr << "Failed to save smoothed loose coupled result: "
                          << appConfig.smoothedOutput << '\n';
                return 1;
            }
            std::cout << "Smoothed loose coupled result saved: "
                      << appConfig.smoothedOutput << '\n';
            if (!smoothedStates.empty())
            {
                const InsState& finalState = smoothedStates.back();
                std::cout << "Final smoothed LC time: " << finalState.time << " s\n";
                std::cout << "Final smoothed LC position: lat " << finalState.latitude * RadToDeg
                          << " deg, lon " << finalState.longitude * RadToDeg
                          << " deg, h " << finalState.height << " m\n";
                std::cout << "Final smoothed LC velocity NED (m/s): "
                          << finalState.velocityNED.transpose() << '\n';
                std::cout << "Final smoothed LC attitude (deg): "
                          << "heading " << finalState.heading * RadToDeg
                          << ", pitch " << finalState.pitch * RadToDeg
                          << ", roll " << finalState.roll * RadToDeg << '\n';
            }
        }
    }

    return 0;
}
