#pragma once

#include <wpi/array.h>

class Constants
{
public:
    static inline wpi::array<double, 5> kStateStdDevs{0.01, 0.01, 0.01, 0.01, 0.01};
    static inline wpi::array<double, 3> kLocalMeasurementStdDevs{0.1, 0.1, 0.1};
    static inline wpi::array<double, 3> kvisionMeasurementStdDevs{0.1, 0.1, 0.1};
};