#pragma once

#include "Vehicle.hpp"
#include "cuda/CudaVehicleUpdater.cuh"
#include "models/IDM.hpp"
#include "road/RoadNetwork.hpp"

#include <cstddef>
#include <vector>

struct VehicleTelemetry
{
    int vehicleId = -1;
    int edgeId = -1;
    int nextEdgeId = -1;
    int destinationNodeId = -1;
    int leaderId = -1;

    float speed = 0.0f;
    float desiredSpeed = 0.0f;
    float acceleration = 0.0f;
    float gap = -1.0f;
};

struct VehicleRoute
{
    int destinationNodeIndex = -1;

    std::vector<int> edgePath;
    std::size_t currentEdgePathIndex = 0;

    int pendingDestinationNodeIndex = -1;
    std::vector<int> pendingEdgePath;

    int completedTrips = 0;
};

class TrafficSimulation
{
public:
    explicit TrafficSimulation(std::size_t vehicleCount = 12, float metersPerVehicle = 10.0f);

    void update(float deltaTime);

    const std::vector<Vehicle>& getVehicles() const;
    const RoadNetwork& getRoadNetwork() const;

    VehicleTelemetry getVehicleTelemetry(std::size_t vehicleIndex) const;

    float getRoadLength() const;
    float getLastKernelTimeMs() const;
    float getLastCudaUpdateTimeMs() const;

private:
    std::size_t findLeaderIndexForTelemetry(std::size_t vehicleIndex) const;

    int chooseDestinationNode(int vehicleId, int startNodeIndex, int tripNumber) const;

    void initializeRoute(std::size_t vehicleIndex);
    void preparePendingRoute(std::size_t vehicleIndex);

    void updateRoutingState();
    void updateRoadTransitions();

    std::vector<Vehicle> vehicles_;
    std::vector<VehicleRoute> vehicleRoutes_;
    std::vector<float> edgeLengths_;

    IDM idm_;
    CudaVehicleUpdater cudaVehicleUpdater_;
    RoadNetwork roadNetwork_;

    float lastCudaUpdateTimeMs_ = 0.0f;

    static constexpr float vehicleLength_ = 4.0f;
};