#pragma once

#include <cstddef>

struct RoadNetworkConfig
{
    int rows = 3;
    int columns = 3;

    float blockLength = 120.0f;
    float laneWidth = 3.5f;

    int lanesPerDirection = 2;
};

namespace RoadNetworkPresets
{
inline constexpr RoadNetworkConfig SMALL{
    3,
    3,
    120.0f,
    3.5f,
    2
};

inline constexpr RoadNetworkConfig MEDIUM{
    6,
    6,
    120.0f,
    3.5f,
    2
};

inline constexpr RoadNetworkConfig LARGE{
    12,
    12,
    150.0f,
    3.5f,
    2
};

inline constexpr RoadNetworkConfig VERY_LARGE{
    20,
    20,
    180.0f,
    3.5f,
    2
};

inline constexpr RoadNetworkConfig MASSIVE{
    30,
    30,
    200.0f,
    3.5f,
    2
};

inline constexpr RoadNetworkConfig choose(std::size_t vehicleCount)
{
    if (vehicleCount <= 50) return SMALL;
    if (vehicleCount <= 1000) return MEDIUM;
    if (vehicleCount <= 20000) return LARGE;
    if (vehicleCount <= 60000) return VERY_LARGE;

    return MASSIVE;
}
}