#pragma once

#include <wpi/array.h>
#include <string>

class Constants
{
public:
    static constexpr double kLoopDt = 0.005;

    static inline wpi::array<double, 5> kStateStdDevs{0.01, 0.01, 0.01, 0.01, 0.01};
    static inline wpi::array<double, 3> kLocalMeasurementStdDevs{0.1, 0.1, 0.1};
    static inline wpi::array<double, 3> kVisionMeasurementStdDevs{0.1, 0.1, 0.1};

    static const int kVisionDataPort = 5800;
    static const std::string kVisionIp;
};
