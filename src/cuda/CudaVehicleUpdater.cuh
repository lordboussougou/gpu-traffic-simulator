#pragma once

#include "simulation/Vehicle.hpp"
#include "simulation/models/IDM.hpp"

#include <cstddef>
#include <vector>

class CudaVehicleUpdater
{
public:
    CudaVehicleUpdater() = default;
    ~CudaVehicleUpdater();

    CudaVehicleUpdater(const CudaVehicleUpdater&) = delete;
    CudaVehicleUpdater& operator=(const CudaVehicleUpdater&) = delete;

    bool update(std::vector<Vehicle>& vehicles, const std::vector<int>& leaderIndices, float deltaTime,
                float roadLength, float vehicleLength, const IDMParameters& idmParameters);

private:
    bool ensureCapacity(std::size_t count);
    void release();

    Vehicle* deviceVehiclesInput_ = nullptr;
    Vehicle* deviceVehiclesOutput_ = nullptr;
    int* deviceLeaderIndices_ = nullptr;
    std::size_t capacity_ = 0;
};
