#pragma once

#include "simulation/Vehicle.hpp"
#include "simulation/models/IDM.hpp"

#include <cstddef>
#include <cuda_runtime_api.h>
#include <vector>

class CudaVehicleUpdater
{
public:
    CudaVehicleUpdater();
    ~CudaVehicleUpdater();

    CudaVehicleUpdater(const CudaVehicleUpdater&) = delete;
    CudaVehicleUpdater& operator=(const CudaVehicleUpdater&) = delete;

    bool update(std::vector<Vehicle>& vehicles, float deltaTime, float roadLength, float vehicleLength,
                const IDMParameters& idmParameters);

    float getLastKernelTimeMs() const;

private:
    bool ensureCapacity(std::size_t count);
    void release();

    Vehicle* deviceVehiclesInput_ = nullptr;
    Vehicle* deviceVehiclesOutput_ = nullptr;

    cudaEvent_t kernelStartEvent_ = nullptr;
    cudaEvent_t kernelStopEvent_ = nullptr;

    std::size_t capacity_ = 0;
    float lastKernelTimeMs_ = 0.0f;
};