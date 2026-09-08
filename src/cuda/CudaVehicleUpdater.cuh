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

    bool update(std::vector<Vehicle>& vehicles, const std::vector<float>& edgeLengths, float deltaTime,
                float vehicleLength, const IDMParameters& idmParameters);

    float getLastKernelTimeMs() const;

private:
    bool ensureCapacity(std::size_t vehicleCount, std::size_t edgeCount);
    void release();

    Vehicle* deviceVehiclesInput_ = nullptr;
    Vehicle* deviceVehiclesOutput_ = nullptr;
    float* deviceEdgeLengths_ = nullptr;

    cudaEvent_t kernelStartEvent_ = nullptr;
    cudaEvent_t kernelStopEvent_ = nullptr;

    std::size_t vehicleCapacity_ = 0;
    std::size_t edgeCapacity_ = 0;

    float lastKernelTimeMs_ = 0.0f;
};