#pragma once

#include "Vehicle.hpp"
#include "cuda/CudaVehicleUpdater.cuh"
#include "models/IDM.hpp"

#include <cstddef>
#include <vector>

struct VehicleTelemetry
{
    int vehicleId = -1;
    int leaderId = -1;

    float speed = 0.0f;
    float desiredSpeed = 0.0f;
    float acceleration = 0.0f;
    float gap = 0.0f;
};

class TrafficSimulation
{
public:
    explicit TrafficSimulation(std::size_t vehicleCount = 12, float metersPerVehicle = 10.0f);

    void update(float deltaTime);

    const std::vector<Vehicle>& getVehicles() const;

    VehicleTelemetry getVehicleTelemetry(std::size_t vehicleIndex) const;

    float getRoadLength() const;
    float getLastKernelTimeMs() const;
    float getLastCudaUpdateTimeMs() const;

private:
    std::size_t findLeaderIndexForTelemetry(std::size_t vehicleIndex) const;
    float distanceAhead(const Vehicle& vehicle, const Vehicle& leader) const;

    std::vector<Vehicle> vehicles_;

    IDM idm_;
    CudaVehicleUpdater cudaVehicleUpdater_;

    float roadLength_ = 200.0f;
    float lastCudaUpdateTimeMs_ = 0.0f;

    static constexpr float vehicleLength_ = 4.0f;
};